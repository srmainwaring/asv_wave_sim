#include "asv_wave_sim_gazebo_plugins/OceanTile.hh"

#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationFFTW.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationOpenCL.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationSimple.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationTrochoid.hh"

// #include <Ogre.h>
// #include <OgreRenderOperation.h>

namespace asv
{
    OceanTile::~OceanTile()
    {
    }

    OceanTile::OceanTile(
    size_t resolution,
    double tileSize) :
        mResolution(resolution),
        mRowLength(resolution + 1),
        mNumVertices((resolution + 1) * (resolution + 1)),
        mNumFaces(2 * resolution * resolution),
        mTileSize(tileSize),
        mSpacing(tileSize / static_cast<double>(resolution)),
        // mWaveSim(resolution, tileSize),
        mHeights(resolution * resolution, 0.0),
        mDhdx(resolution * resolution, 0.0),
        mDhdy(resolution * resolution, 0.0),
        mDisplacementsX(resolution * resolution, 0.0),
        mDisplacementsY(resolution * resolution, 0.0),
        mDxdx(resolution * resolution, 0.0),
        mDydy(resolution * resolution, 0.0),
        mDxdy(resolution * resolution, 0.0)
    {
        // Different types of wave simulator are supported...

        // 1. FFTW / OpenCL
        #if 0

        // mWaveSim.reset(new WaveSimulationOpenCL(resolution, tileSize));
        mWaveSim.reset(new WaveSimulationFFTW(resolution, tileSize));

        #else

        // 2. Trochoid
        // std::shared_ptr<WaveParameters> waveParams(new WaveParameters());
        // waveParams->SetNumber(3);
        // waveParams->SetAngle(0.6);
        // waveParams->SetScale(1.2);
        // waveParams->SetSteepness(1.0);
        // waveParams->SetAmplitude(1.0);
        // waveParams->SetPeriod(7.0);
        // waveParams->SetDirection(Vector2(1.0, 0.0));

        // mWaveSim.reset(new WaveSimulationTrochoid(resolution, tileSize, waveParams));

        // 3. Simple
        mWaveSim.reset(new WaveSimulationSimple(resolution, tileSize));

        #endif
    }

    void OceanTile::setWindVelocity(double ux, double uy)
    {
        mWaveSim->SetWindVelocity(ux, uy);
    }
    
    // See:
    //  osgOcean/OceanTile.
    //  ocean_gazebo_plugins/ShaderVisual.cc
    //
    // The texture coordinates (u,v) span the entire tile.
    // The Ogre convention for texture coordinates has:
    // (u, v) = (0, 0) at the top left 
    // (u, v) = (1, 1) at the bottom right 
    // The tangent space basis calculation is adjusted to 
    // conform with this convention.
    void OceanTile::create()
    {
        // auto&& logManager = Ogre::LogManager::getSingleton();
        // logManager.logMessage("Creating OceanTile...");
        // logManager.logMessage("Resolution:    " + Ogre::StringConverter::toString(mResolution));
        // logManager.logMessage("RowLength:     " + Ogre::StringConverter::toString(mRowLength));
        // logManager.logMessage("NumVertices:   " + Ogre::StringConverter::toString(mNumVertices));
        // logManager.logMessage("NumFaces:      " + Ogre::StringConverter::toString(mNumFaces));
        // logManager.logMessage("TileSize:      " + Ogre::StringConverter::toString(mTileSize));
        // logManager.logMessage("Spacing:       " + Ogre::StringConverter::toString(mSpacing));

        // Grid dimensions
        const size_t nx = this->mResolution;
        const size_t ny = this->mResolution;
        const double Lx = this->mTileSize;
        const double Ly = this->mTileSize;
        const double lx = this->mSpacing;
        const double ly = this->mSpacing;
        const double xTex = 1.0 * lx;
        const double yTex = 1.0 * ly;

        // Vertices - (N+1) vertices in each row / column 
        for (size_t iy=0; iy<=ny; ++iy)
        {
            double py = iy * ly - Ly/2.0;
            for (size_t ix=0; ix<=nx; ++ix)
            {
                // Vertex position
                double px = ix * lx - Lx/2.0;
                Ogre::Vector3 vertex(px, py, 0);        
                mVertices0.push_back(vertex);
                mVertices.push_back(vertex);

                // Texture coordinates (u, v): top left: (0, 0), bottom right: (1, 1)
                Ogre::Vector2 texCoord(ix * xTex, 1.0 - (iy * yTex));
                mTexCoords.push_back(texCoord);
            }
        }

        // Indices
        for (size_t iy=0; iy<ny; ++iy)
        {
            for (size_t ix=0; ix<nx; ++ix)
            {
                // Get the vertices in the cell coordinates
                const size_t idx0 = iy * (nx+1) + ix;
                const size_t idx1 = iy * (nx+1) + ix + 1;
                const size_t idx2 = (iy+1) * (nx+1) + ix + 1;
                const size_t idx3 = (iy+1) * (nx+1) + ix;

                // Indices
                mFaces.push_back(ignition::math::Vector3i(idx0, idx1, idx2));
                mFaces.push_back(ignition::math::Vector3i(idx0, idx2, idx3));
            }
        }

        // Texture Coordinates
        mTangents.assign(mVertices.size(), Ogre::Vector3::ZERO);
        mBitangents.assign(mVertices.size(), Ogre::Vector3::ZERO);
        mNormals.assign(mVertices.size(), Ogre::Vector3::ZERO);
        
        computeNormals();
        computeTangentSpace();
        createMesh("OceanTileMesh");
    }

    void OceanTile::computeNormals()
    {
        // auto&& logManager = Ogre::LogManager::getSingleton();
        // logManager.logMessage("Computing normals...");

        // 0. Reset normals.
        mNormals.assign(mVertices.size(), Ogre::Vector3::ZERO);

        // 1. For each face calculate the normal and add to each vertex in the face
        for (size_t i=0; i<mNumFaces; ++i)
        {
            // Vertices
            auto v0Idx = mFaces[i][0];
            auto v1Idx = mFaces[i][1];
            auto v2Idx = mFaces[i][2];
            auto&& v0 = mVertices[v0Idx];
            auto&& v1 = mVertices[v1Idx];
            auto&& v2 = mVertices[v2Idx];

            // Normal
            Ogre::Vector3 normal(Ogre::Math::calculateBasicFaceNormal(v0, v1, v2));
    
            // Add to vertices
            mNormals[v0Idx] += normal;
            mNormals[v1Idx] += normal;
            mNormals[v2Idx] += normal;
        }

        // 2. Normalise each vertex normal.
        for (auto&& normal : mNormals)
        {
            normal.normalise();
        }

        // logManager.logMessage("Normals computed.");
    }

    void OceanTile::computeTangentSpace()
    {
        // auto&& logManager = Ogre::LogManager::getSingleton();
        // logManager.logMessage("Computing tangent space...");

        computeTBN(mVertices, mTexCoords, mFaces, mTangents, mBitangents, mNormals);

        // @DEBUG
        #if 0
        for (size_t i=0; i<std::min(static_cast<size_t>(20), mVertices.size()) ; ++i)
        {
            logManager.logMessage("V["
            + Ogre::StringConverter::toString(i) + "]:  "
            + Ogre::StringConverter::toString(mVertices[i]));
            logManager.logMessage("UV["
            + Ogre::StringConverter::toString(i) + "]: "
            + Ogre::StringConverter::toString(mTexCoords[i]));
            logManager.logMessage("T["
            + Ogre::StringConverter::toString(i) + "]:  "
            + Ogre::StringConverter::toString(mTangents[i]));
            logManager.logMessage("B["
            + Ogre::StringConverter::toString(i) + "]:  "
            + Ogre::StringConverter::toString(mBitangents[i]));
            logManager.logMessage("N["
            + Ogre::StringConverter::toString(i) + "]:  "
            + Ogre::StringConverter::toString(mNormals[i]));
            logManager.logMessage("");
        }
        #endif

        // logManager.logMessage("Tangent space computed.");
    }

    // Compute the tangent space vectors (Tanget, Bitangent, Normal)
    //
    // Adapted from:
    // https://learnopengl.com/Advanced-Lighting/Normal-Mapping
    // Retrieved: 03 April 2019
    //
    // Resources:
    // Learn OpenGL: https://learnopengl.com/Advanced-Lighting/Normal-Mapping
    // Bumpmapping with GLSL: http://fabiensanglard.net/bumpMapping/index.php
    // Lesson 8: Tangent Space: http://jerome.jouvie.free.fr/opengl-tutorials/Lesson8.php
    //
    void OceanTile::computeTBN(
        const Ogre::Vector3& p0, 
        const Ogre::Vector3& p1, 
        const Ogre::Vector3& p2, 
        const Ogre::Vector2& uv0, 
        const Ogre::Vector2& uv1, 
        const Ogre::Vector2& uv2, 
        Ogre::Vector3& tangent, 
        Ogre::Vector3& bitangent, 
        Ogre::Vector3& normal)
    {
        // Correction to the TBN calculation when the v texture coordinate
        // is 0 at the top of a texture and 1 at the bottom. 
        double vsgn = -1.0;
        auto edge1 = p1 - p0;
        auto edge2 = p2 - p0;
        auto duv1 = uv1 - uv0;
        auto duv2 = uv2 - uv0;

        double f = 1.0f / (duv1.x * duv2.y - duv2.x * duv1.y) * vsgn;

        tangent.x = f * (duv2.y * edge1.x - duv1.y * edge2.x) * vsgn;
        tangent.y = f * (duv2.y * edge1.y - duv1.y * edge2.y) * vsgn;
        tangent.z = f * (duv2.y * edge1.z - duv1.y * edge2.z) * vsgn;
        tangent.normalise();

        bitangent.x = f * (-duv2.x * edge1.x + duv1.x * edge2.x);
        bitangent.y = f * (-duv2.x * edge1.y + duv1.x * edge2.y);
        bitangent.z = f * (-duv2.x * edge1.z + duv1.x * edge2.z);
        bitangent.normalise();  

        normal = tangent.crossProduct(bitangent);
        normal.normalise();  
    }

    void OceanTile::computeTBN(
        const std::vector<Ogre::Vector3>& vertices,
        const std::vector<Ogre::Vector2>& texCoords,
        const std::vector<ignition::math::Vector3i>& faces, 
        std::vector<Ogre::Vector3>& tangents,
        std::vector<Ogre::Vector3>& bitangents,
        std::vector<Ogre::Vector3>& normals)
    {
        // 0. Resize and zero outputs.
        tangents.assign(vertices.size(), Ogre::Vector3::ZERO);
        bitangents.assign(vertices.size(), Ogre::Vector3::ZERO);
        normals.assign(vertices.size(), Ogre::Vector3::ZERO);

        // 1. For each face calculate TBN and add to each vertex in the face.
        for (auto&& face : faces)
        {
            // Face vertex indices.
            auto idx0 = face[0];
            auto idx1 = face[1];
            auto idx2 = face[2];

            // Face vertex points.
            auto&& p0 = vertices[idx0];    
            auto&& p1 = vertices[idx1];    
            auto&& p2 = vertices[idx2];    

            // Face vertex texture coordinates.
            auto&& uv0 = texCoords[idx0];
            auto&& uv1 = texCoords[idx1];
            auto&& uv2 = texCoords[idx2];

            // Compute tangent space.
            Ogre::Vector3 T, B, N;
            computeTBN(p0, p1, p2, uv0, uv1, uv2, T, B, N);

            // Assign to vertices.
            for (int i=0; i<3; ++i)
            {
                auto idx = face[i];
                tangents[idx]   += T;
                bitangents[idx] += B;
                normals[idx]    += N;
            }
        } 

        // 2. Normalise each vertex's tangent space basis.
        for (size_t i=0; i<vertices.size(); ++i)
        {
            tangents[i].normalise();
            bitangents[i].normalise();
            normals[i].normalise();
        }
    }

    void OceanTile::update(double time)
    {
        updateVertices(time);
        // Uncomment to calculate the tangent space using finite differences
        // computeTangentSpace();
        updateMesh();
    }

    void OceanTile::updateVertices(double time)
    {
        // Logging
        // auto& logManager = Ogre::LogManager::getSingleton();
        // logManager.logMessage("Updating vertices...");

        // logManager.logMessage("Set time");
        mWaveSim->SetTime(time);

        // logManager.logMessage("Compute heights");
        mWaveSim->ComputeDisplacementsAndDerivatives(
            mHeights, mDisplacementsX, mDisplacementsY,
            mDhdx, mDhdy, mDxdx, mDydy, mDxdy);

        // mWaveSim->ComputeHeights(mHeights);
        // for (auto&& h : mHeights)
        // {
        //   std::cout << h << std::endl;
        // }
        // std::cout << std::endl;

        // logManager.logMessage("Compute displacements");
        // mWaveSim->ComputeDisplacements(mDisplacementsX, mDisplacementsY);

        // @TODO_MOVE Set displacement scaling and check signs
        // double lambda = 0.9;
        const size_t N = mResolution;
        const size_t NPlus1 = N + 1;
        const size_t NMinus1 = N - 1;

        for (size_t iy=0; iy<N; ++iy)
        {
            for (size_t ix=0; ix<N; ++ix)
            {
                size_t idx0 =  iy * NPlus1 + ix;
                size_t idx1 =  iy * N + ix;

                double h  = mHeights[idx1];
                double sx = mDisplacementsX[idx1];
                double sy = mDisplacementsY[idx1];

                auto&& v0 = mVertices0[idx0];
                auto&& v  = mVertices[idx0];
                v.x = v0.x + sx;
                v.y = v0.y + sy;
                v.z = v0.z + h;
        //     }
        // }

        // logManager.logMessage("Compute height derivatives");
        // mWaveSim->ComputeHeightDerivatives(mDhdx, mDhdy);

        // logManager.logMessage("Compute displacement derivatives");
        // mWaveSim->ComputeDisplacementDerivatives(mDxdx, mDydy, mDxdy);

        // 0. Resize and zero outputs.
        // mTangents.assign(mVertices.size(), Ogre::Vector3::ZERO);
        // mBitangents.assign(mVertices.size(), Ogre::Vector3::ZERO);
        // mNormals.assign(mVertices.size(), Ogre::Vector3::ZERO);

        // 1. Update tangent and bitangent vectors (not normalised).
        // @TODO Check sign for displacement terms
        // for (size_t iy=0; iy<N; ++iy)
        // {
        //     for (size_t ix=0; ix<N; ++ix)
        //     {
                // size_t idx0 =  iy * NPlus1 + ix;
                // size_t idx1 =  iy * N + ix;
                double dhdx  = mDhdx[idx1]; 
                double dhdy  = mDhdy[idx1]; 
                double dsxdx = mDxdx[idx1]; 
                double dsydy = mDydy[idx1]; 
                double dsxdy = mDxdy[idx1]; 

                auto&& t = mTangents[idx0];
                t.x = dsxdx + 1.0;
                t.y = dsxdy;
                t.z = dhdx;      
                
                auto&& b = mBitangents[idx0];
                b.x = dsxdy;
                b.y = dsydy + 1.0;
                b.z = dhdy;       
            }
        }

        // Set skirt values
        // logManager.logMessage("Compute tile skirt vertices");
        for (size_t i=0; i<=N; ++i)
        {
            size_t idx0 =  N * NPlus1 + i;
            size_t idx1 =  NMinus1 * NPlus1 + i;
            mVertices[idx0] = mVertices[idx1];
            mTangents[idx0] = mTangents[idx1];
            mBitangents[idx0] = mBitangents[idx1];
        // }
        // for (size_t iy=0; iy<=N; ++iy)
        // {
            size_t idy0 =  i * NPlus1 + N;
            size_t idy1 =  i * NPlus1 + NMinus1;
            mVertices[idy0] = mVertices[idy1];
            mTangents[idy0] = mTangents[idy1];
            mBitangents[idy0] = mBitangents[idy1];
        }

        // logManager.logMessage("Done updating vertices.");
    }

    void OceanTile::createMesh(const Ogre::String& name)
    {
        // Logging
        auto& logManager = Ogre::LogManager::getSingleton();
        logManager.logMessage("Creating OceanTile mesh...");

        // Create mesh
        // @NOTE  Cannot hold a reference to the mesh pointer in the class
        //        otherwise there will be a seg. fault on exit (ownership issue?).
        Ogre::MeshPtr mMesh = Ogre::MeshManager::getSingleton().createManual(name, "General");

        // Create submesh
        mSubMesh = mMesh->createSubMesh();

        // Vertices
        const size_t nVertices = mVertices.size();
        const size_t posVertexBufferCount = (3 * 2) * nVertices;
        const size_t texVertexBufferCount = (3 * 2 + 2) * nVertices;

        // Indices (orientation must be counter-clockwise for normals to be correct)
        const size_t nFaces = mFaces.size();
        const size_t indexBufferCount = 3 * nFaces;

        // Hardware buffer manager
        auto& hardwareBufferManager = Ogre::HardwareBufferManager::getSingleton();

        // Create vertex data (also creates vertexDeclaration and vertexBufferBinding)
        mSubMesh->vertexData = new Ogre::VertexData();
        mSubMesh->vertexData->vertexCount = nVertices;
        mSubMesh->vertexData->vertexStart = 0;

        // Create vertex declaration: positions, normals
        unsigned int posVertexBufferIndex = 0;
        size_t offset = 0;
        mSubMesh->vertexData->vertexDeclaration->addElement(
            posVertexBufferIndex, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

        mSubMesh->vertexData->vertexDeclaration->addElement(
            posVertexBufferIndex, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3); 

        // Create vertex declaration: texture coordinates, tangents, bitangents
        unsigned int texVertexBufferIndex = 1;
        offset = 0;
        // TexCoords: uv0
        mSubMesh->vertexData->vertexDeclaration->addElement(
            texVertexBufferIndex, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2); 

        // Tangents: uv6
        mSubMesh->vertexData->vertexDeclaration->addElement(
            texVertexBufferIndex, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES, 6);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3); 

        // Bitangents: uv7
        mSubMesh->vertexData->vertexDeclaration->addElement(
            texVertexBufferIndex, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES, 7);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3); 

        // Allocate vertex buffer: positions
        auto posVertexBuffer =
            hardwareBufferManager.createVertexBuffer(
                mSubMesh->vertexData->vertexDeclaration->getVertexSize(posVertexBufferIndex),
                mSubMesh->vertexData->vertexCount,
                Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE,
                true);

        // Allocate vertex buffer: textures
        auto texVertexBuffer =
            hardwareBufferManager.createVertexBuffer(
                mSubMesh->vertexData->vertexDeclaration->getVertexSize(texVertexBufferIndex),
                mSubMesh->vertexData->vertexCount,
                Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE,
                true);

        // Set vertex buffer bindings
        mSubMesh->vertexData->vertexBufferBinding->setBinding(posVertexBufferIndex, posVertexBuffer);
        mSubMesh->vertexData->vertexBufferBinding->setBinding(texVertexBufferIndex, texVertexBuffer);

        // Lock vertex buffers for write
        float* gpuPosVertices = static_cast<float*>(
            posVertexBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
        float* gpuTexVertices = static_cast<float*>(
            texVertexBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));

        // Allocate index buffer of the requested number of vertices (ibufCount) 
        auto indexBuffer = 
            hardwareBufferManager.createIndexBuffer(
                Ogre::HardwareIndexBuffer::IT_32BIT, 
                indexBufferCount, 
                Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                false);

        // Lock index buffer for write
        uint32_t *gpuIndices = static_cast<uint32_t*>(
            indexBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));    

        // Set submesh index parameters
        mSubMesh->useSharedVertices = false;
        mSubMesh->indexData->indexBuffer = indexBuffer;
        mSubMesh->indexData->indexCount = indexBufferCount;
        mSubMesh->indexData->indexStart = 0;

        // Copy position vertices to GPU
        for (size_t i=0; i<nVertices; ++i)
        {
            *gpuPosVertices++ = mVertices[i][0];
            *gpuPosVertices++ = mVertices[i][1];
            *gpuPosVertices++ = mVertices[i][2];

            *gpuPosVertices++ = mNormals[i][0];
            *gpuPosVertices++ = mNormals[i][1];
            *gpuPosVertices++ = mNormals[i][2];
        // }

        // Copy texture vertices to GPU
        // for (size_t i=0; i<nVertices; ++i)
        // {
            // uv0
            *gpuTexVertices++ = mTexCoords[i][0];
            *gpuTexVertices++ = mTexCoords[i][1];

            // uv6
            *gpuTexVertices++ = mTangents[i][0];
            *gpuTexVertices++ = mTangents[i][1];
            *gpuTexVertices++ = mTangents[i][2];

            // uv7
            *gpuTexVertices++ = mBitangents[i][0];
            *gpuTexVertices++ = mBitangents[i][1];
            *gpuTexVertices++ = mBitangents[i][2];
        }

        // Copy indices to GPU
        for (size_t i=0; i<nFaces; ++i)
        {
            *gpuIndices++ = mFaces[i][0];
            *gpuIndices++ = mFaces[i][1];
            *gpuIndices++ = mFaces[i][2];
        }

        // Unlock buffers
        posVertexBuffer->unlock();
        texVertexBuffer->unlock();

        indexBuffer->unlock();

        // Set bounds (box and sphere)
        mMesh->_setBounds(Ogre::AxisAlignedBox(
        -mTileSize, -mTileSize, -mTileSize,
        mTileSize,  mTileSize,  mTileSize));
        mMesh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3.0 * mTileSize * mTileSize));

        // Load mesh
        mMesh->load();

        logManager.logMessage("OceanTile mesh created.");
    }

    void OceanTile::updateMesh()
    {
        // Logging
        // auto& logManager = Ogre::LogManager::getSingleton();
        // logManager.logMessage("Updating OceanTile mesh...");

        // Retrieve vertexData
        auto vertexData = mSubMesh->vertexData;
        
        // Get position vertex buffer and obtain lock for writing.
        auto posElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
        auto posVertexBuffer = vertexData->vertexBufferBinding->getBuffer(posElement->getSource());
        float* gpuPosVertices = static_cast<float*>(
            posVertexBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));

        // Get texcoord vertex buffer and obtain lock for writing.
        auto texElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES, 0);
        auto texVertexBuffer = vertexData->vertexBufferBinding->getBuffer(texElement->getSource());
        float* gpuTexVertices = static_cast<float*>(
            texVertexBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));

        // Copy position vertices to GPU
        for (size_t i=0; i<mVertices.size(); ++i)
        {
            *gpuPosVertices++ = mVertices[i][0];
            *gpuPosVertices++ = mVertices[i][1];
            *gpuPosVertices++ = mVertices[i][2];

            *gpuPosVertices++ = mNormals[i][0];
            *gpuPosVertices++ = mNormals[i][1];
            *gpuPosVertices++ = mNormals[i][2];
        // }

        // Copy texture vertices to GPU
        // for (size_t i=0; i<mVertices.size(); ++i)
        // {
            *gpuTexVertices++ = mTexCoords[i][0];
            *gpuTexVertices++ = mTexCoords[i][1];

            *gpuTexVertices++ = mTangents[i][0];
            *gpuTexVertices++ = mTangents[i][1];
            *gpuTexVertices++ = mTangents[i][2];

            *gpuTexVertices++ = mBitangents[i][0];
            *gpuTexVertices++ = mBitangents[i][1];
            *gpuTexVertices++ = mBitangents[i][2];
        }

        // Unlock buffers
        posVertexBuffer->unlock();
        texVertexBuffer->unlock();

        // Set bounds (box and sphere)
        // mMesh->_setBounds(Ogre::AxisAlignedBox(
        //   -0.5 * mTileSize, -0.5 * mTileSize, -0.5 * mTileSize,
        //    0.5 * mTileSize,  0.5 * mTileSize,  0.5 * mTileSize));
        // mMesh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3.0 * 0.5 * 0.5 * mTileSize * mTileSize));

        // Load mesh
        // mMesh->load();

        // logManager.logMessage("OceanTile mesh updated.");    
    }

    void OceanTile::debugPrintVertexBuffers() const
    {
        // Logging
        auto& logManager = Ogre::LogManager::getSingleton();
        logManager.logMessage("DEBUG - READING VERTEX BUFFER...");    
        auto vertexData = mSubMesh->vertexData;
        logManager.logMessage("Use shared vertices: "
            + Ogre::StringConverter::toString(mSubMesh->useSharedVertices));    
        
        // VES_POSITION
        {
            auto element = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
            auto vertexBuffer = vertexData->vertexBufferBinding->getBuffer(element->getSource());
            auto vertex = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            logManager.logMessage("Source: " + Ogre::StringConverter::toString(element->getSource()));    

            for (size_t i=0; i<vertexData->vertexCount; ++i, vertex += vertexBuffer->getVertexSize())
            {
                Ogre::Real* real;
                element->baseVertexPointerToElement(vertex, &real);

                Ogre::Vector3 data;
                data.x = (*real++);
                data.y = (*real++);
                data.z = (*real++);

                logManager.logMessage("Position[" 
                    + Ogre::StringConverter::toString(i)
                    + "]: "
                    + Ogre::StringConverter::toString(data)
                    );
            }
            vertexBuffer->unlock();
        }
        
        // VES_NORMAL
        {
            auto element = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
            auto vertexBuffer = vertexData->vertexBufferBinding->getBuffer(element->getSource());
            auto vertex = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            logManager.logMessage("Source: " + Ogre::StringConverter::toString(element->getSource()));    

            for (size_t i=0; i<vertexData->vertexCount; ++i, vertex += vertexBuffer->getVertexSize())
            {
                Ogre::Real* real;
                element->baseVertexPointerToElement(vertex, &real);

                Ogre::Vector3 data;
                data.x = (*real++);
                data.y = (*real++);
                data.z = (*real++);

                logManager.logMessage("Normal[" 
                    + Ogre::StringConverter::toString(i)
                    + "]: "
                    + Ogre::StringConverter::toString(data)
                    );
            }
            vertexBuffer->unlock();
        }

        // VES_TEXTURE_COORDINATES
        {
            auto element = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES, 0);
            auto vertexBuffer = vertexData->vertexBufferBinding->getBuffer(element->getSource());
            auto vertex = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            logManager.logMessage("Source: " + Ogre::StringConverter::toString(element->getSource()));    

            for (size_t i=0; i<vertexData->vertexCount; ++i, vertex += vertexBuffer->getVertexSize())
            {
                Ogre::Real* real;
                element->baseVertexPointerToElement(vertex, &real);

                Ogre::Vector2 data;
                data.x = (*real++);
                data.y = (*real++);

                logManager.logMessage("UV0[" 
                    + Ogre::StringConverter::toString(i)
                    + "]: "
                    + Ogre::StringConverter::toString(data)
                    );
            }
            vertexBuffer->unlock();
        }

        // VES_TEXTURE_COORDINATES (TANGENT)
        {
            auto element = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES, 6);
            auto vertexBuffer = vertexData->vertexBufferBinding->getBuffer(element->getSource());
            auto vertex = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            logManager.logMessage("Source: " + Ogre::StringConverter::toString(element->getSource()));    

            for (size_t i=0; i<vertexData->vertexCount; ++i, vertex += vertexBuffer->getVertexSize())
            {
                Ogre::Real* real;
                element->baseVertexPointerToElement(vertex, &real);

                Ogre::Vector3 data;
                data.x = (*real++);
                data.y = (*real++);
                data.z = (*real++);

                logManager.logMessage("T=UV6[" 
                    + Ogre::StringConverter::toString(i)
                    + "]: "
                    + Ogre::StringConverter::toString(data)
                    );
            }
            vertexBuffer->unlock();
        }

        // VES_TEXTURE_COORDINATES (BITANGENT)
        {
            auto element = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES, 7);
            auto vertexBuffer = vertexData->vertexBufferBinding->getBuffer(element->getSource());
            auto vertex = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            logManager.logMessage("Source: " + Ogre::StringConverter::toString(element->getSource()));    

            for (size_t i=0; i<vertexData->vertexCount; ++i, vertex += vertexBuffer->getVertexSize())
            {
                Ogre::Real* real;
                element->baseVertexPointerToElement(vertex, &real);

                Ogre::Vector3 data;
                data.x = (*real++);
                data.y = (*real++);
                data.z = (*real++);

                logManager.logMessage("B=UV7[" 
                    + Ogre::StringConverter::toString(i)
                    + "]: "
                    + Ogre::StringConverter::toString(data)
                    );
            }
            vertexBuffer->unlock();
        }

        logManager.logMessage("VERTEX BUFFER READ.");
    
    }

}
