// Copyright (C) 2021  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// The DynamicGeometry example is sourced from the Ogre2 Samples 

// OGRE (www.ogre3d.org) is made available under the MIT License.
//
// Copyright (c) 2000-2013 Torus Knot Software Ltd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "DynamicGeometry.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/ShaderParams.hh>
#include <ignition/rendering/Visual.hh>

#include <ignition/rendering/Grid.hh>

#include <ignition/rendering/ogre2.hh>
#include <ignition/rendering/ogre2/Ogre2MeshFactory.hh>
#include <ignition/rendering/ogre2/Ogre2Scene.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Element.hh>

// Ogre2
#include <OgreItem.h>
#include <OgreMesh2.h>
#include <OgreMeshManager2.h>
#include <OgreSceneManager.h>
#include <OgreSubMesh2.h>
#include <Vao/OgreVaoManager.h>
// Ogre2

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

  //////////////////////////////////////////////////

  // Subclass from Ogre2Mesh and Ogre2MeshFactory to get
  // indirect access to protected members and override any
  // behaviour that tries to load a common::Mesh which we
  // are not using.

  //////////////////////////////////////////////////
  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshExt :
      public Ogre2Mesh
  {
    /// \brief Destructor
    public: virtual ~Ogre2MeshExt() {}

    /// \brief Constructor
    protected: explicit Ogre2MeshExt() : Ogre2Mesh() {}

    /// \brief Allow intercept of pre-render call for this mesh
    public: virtual void PreRender() override
    {
      Ogre2Mesh::PreRender();
    }

    /// \brief Work-around the protected accessors and protected methods in Scene
    public: void InitObject(Ogre2ScenePtr _scene, unsigned int _id, const std::string &_name)
    {
      this->id = _id;
      this->name = _name;
      this->scene = _scene;

      // initialize object
      this->Load();
      this->Init();
    }

    /// \brief Used by friend class (Ogre2MeshFactoryExt)
    protected: void SetOgreItem(Ogre::Item *_ogreItem)
    {
      this->ogreItem = _ogreItem;
    }

    private: friend class Ogre2MeshFactoryExt;
  };

  typedef std::shared_ptr<Ogre2MeshExt> Ogre2MeshExtPtr;

  //////////////////////////////////////////////////
  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshFactoryExt :
      public Ogre2MeshFactory
  {
    /// \brief Destructor
    public: virtual ~Ogre2MeshFactoryExt() {}

    /// \brief Constructor - construct from an Ogre2ScenePtr
    public: explicit Ogre2MeshFactoryExt(Ogre2ScenePtr _scene) :
        Ogre2MeshFactory(_scene), ogre2Scene(_scene) {}

    /// \brief Override - use an extension of Ogre2Mesh
    public: virtual Ogre2MeshPtr Create(const MeshDescriptor &_desc) override
    {
      // create ogre entity
      Ogre2MeshExtPtr mesh(new Ogre2MeshExt);
      MeshDescriptor normDesc = _desc;
      // \todo do this? override MeshDescriptor behaviour as we're not using common::Mesh
      normDesc.Load();
      mesh->SetOgreItem(this->OgreItem(normDesc));

      // check if invalid mesh
      if (!mesh->ogreItem)
      {
        ignerr << "Failed to get Ogre item for [" << _desc.meshName << "]"
                << std::endl;
        return nullptr;
      }

      // create sub-mesh store
      Ogre2SubMeshStoreFactory subMeshFactory(this->scene, mesh->ogreItem);
      mesh->subMeshes = subMeshFactory.Create();
      for (unsigned int i = 0; i < mesh->subMeshes->Size(); i++)
      {
        Ogre2SubMeshPtr submesh =
            std::dynamic_pointer_cast<Ogre2SubMesh>(mesh->subMeshes->GetById(i));
        submesh->SetMeshName(this->MeshName(_desc));
      }
      return mesh;
    }

    /// \brief Override \todo: may be able to use base class implementation...
    protected: virtual Ogre::Item * OgreItem(const MeshDescriptor &_desc) override
    {
      if (!this->Load(_desc))
      {
        return nullptr;
      }

      std::string name = this->MeshName(_desc);
      ignmsg << "Get Ogre::SceneManager\n";
      Ogre::SceneManager *sceneManager = this->scene->OgreSceneManager();

      ignmsg << "Check for v2 mesh\n";
      // check if a v2 mesh already exists
      Ogre::MeshPtr mesh =
          Ogre::MeshManager::getSingleton().getByName(name);

      // if not, it probably has not been imported from v1 yet
      if (!mesh)
      {
        ignmsg << "Check for v1 mesh\n";
        Ogre::v1::MeshPtr v1Mesh =
            Ogre::v1::MeshManager::getSingleton().getByName(name);
        if (!v1Mesh)
        {
          ignerr << "Did not find v1 mesh [" << name << "]\n";
          return nullptr;
        }

        // examine v1 mesh properties
        v1Mesh->load();
        ignmsg << "v1 mesh: isLoaded: " << v1Mesh->isLoaded() << "\n";

        ignmsg << "Creating v2 mesh\n";
        // create v2 mesh from v1
        mesh = Ogre::MeshManager::getSingleton().createManual(
            name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        ignmsg << "Importing v2 mesh\n";
        mesh->importV1(v1Mesh.get(), false, true, true);
        this->ogreMeshes.push_back(name);
      }
      else
      {
        ignmsg << "Found v2 mesh\n";
      }

      return sceneManager->createItem(mesh, Ogre::SCENE_DYNAMIC);
    }

    /// \brief Override
    protected: virtual bool LoadImpl(const MeshDescriptor &_desc) override
    {
      /// \todo: currently assume we've already loaded from the tile
      /// but should handle better
      return true;
    }

    /// \brief Override \todo: may be able to use base class implementation...
    public: virtual bool Validate(const MeshDescriptor &_desc) override
    {
      if (!_desc.mesh && _desc.meshName.empty())
      {
        ignerr << "Invalid mesh-descriptor, no mesh specified" << std::endl;
        return false;
      }

      if (!_desc.mesh)
      {
        ignerr << "Cannot load null mesh [" << _desc.meshName << "]" << std::endl;
        return false;
        // Override MeshDescriptor behaviour as we're not using common::Mesh
        // return true;
      }

      if (_desc.mesh->SubMeshCount() == 0)
      {
        ignerr << "Cannot load mesh with zero sub-meshes" << std::endl;
        return false;
      }

      return true;
    }

    /// \brief Pointer to the derived Ogre2Scene
    private: rendering::Ogre2ScenePtr ogre2Scene;

  };

  typedef std::shared_ptr<Ogre2MeshFactoryExt> Ogre2MeshFactoryExtPtr;
}
}
}

using namespace ignition;
using namespace gazebo;
using namespace systems;

// From ogre-next/Samples/2.0/ApiUsage/DynamicGeometry/DynamicGeometryGameState.h/cpp

struct ignition::gazebo::systems::CubeVertices
{
    float px, py, pz;   //Position
    float nx, ny, nz;   //Normals

    CubeVertices() {}
    CubeVertices( float _px, float _py, float _pz,
                  float _nx, float _ny, float _nz ) :
        px( _px ), py( _py ), pz( _pz ),
        nx( _nx ), ny( _ny ), nz( _nz )
    {
    }
};

const CubeVertices c_originalVertices[8] =
{
    CubeVertices( -1, -1,  1, -0.57737, -0.57737,  0.57737 ),
    CubeVertices(  1, -1,  1,  0.57737, -0.57737,  0.57737 ),
    CubeVertices(  1,  1,  1,  0.57737,  0.57737,  0.57737 ),
    CubeVertices( -1,  1,  1, -0.57737,  0.57737,  0.57737 ),
    CubeVertices( -1, -1, -1, -0.57737, -0.57737, -0.57737 ),
    CubeVertices(  1, -1, -1,  0.57737, -0.57737, -0.57737 ),
    CubeVertices(  1,  1, -1,  0.57737,  0.57737, -0.57737 ),
    CubeVertices( -1,  1, -1, -0.57737,  0.57737, -0.57737 )
};

class ignition::gazebo::systems::DynamicGeometryPrivate
{
  /// \brief Path to the model
  public: std::string modelPath;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: ignition::common::ConnectionPtr connection{nullptr};

  /// \brief Name of visual this plugin is attached to
  public: std::string visualName;

  /// \brief Pointer to visual
  public: rendering::VisualPtr visual;

  /// \brief Pointer to dynamic visual
  public: rendering::VisualPtr dynamicVisual;

  /// \brief Pointer to scene
  public: rendering::ScenePtr scene;

  /// \brief Entity id of the visual
  public: Entity entity = kNullEntity;

  /// \brief Current sim time
  public: std::chrono::steady_clock::duration currentSimTime;

  /// \brief Destructor
  public: ~DynamicGeometryPrivate();

  /// \brief Constructor
  public: DynamicGeometryPrivate();

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();

  /////////////////
  // From ogre-next/Samples/2.0/ApiUsage/DynamicGeometry/DynamicGeometryGameState.h/cpp
  Ogre::MeshPtr               mStaticMesh;
  Ogre::MeshPtr               mPartialMesh;
  Ogre::MeshPtr               mDynamicMesh[2];
  Ogre::VertexBufferPacked    *mDynamicVertexBuffer[2];

  float mRotationTime;
  
  /// Helper function to retrieve the render system
  Ogre::RenderSystem * RenderSystem() const;

  /// Helper function to create an index buffer.
  Ogre::IndexBufferPacked * CreateIndexBuffer(void);

  /// Creates the MeshPtr needed by mStaticMesh & mPartialMesh
  Ogre::MeshPtr CreateStaticMesh(bool partialMesh);

  /// Creates a dynamic buffer. The idx parameter is needed for the mesh' name
  std::pair<Ogre::MeshPtr, Ogre::VertexBufferPacked*> CreateDynamicMesh(size_t idx);

  /// Helper function to fill the 2nd dynamic vertex buffer
  void UpdateDynamicBuffer01(float *cubeVertices,
      const CubeVertices originalVerts[8],
      size_t start, size_t end) const;

  void Update(float simTime);

  void CreateScene01(void);
  
  void DestroyScene();
  /////////////////
};

/////////////////////////////////////////////////
DynamicGeometry::DynamicGeometry()
    : System(), dataPtr(std::make_unique<DynamicGeometryPrivate>())
{
}

/////////////////////////////////////////////////
DynamicGeometry::~DynamicGeometry()
{
}

/////////////////////////////////////////////////
void DynamicGeometry::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  IGN_PROFILE("DynamicGeometry::Configure");

  ignmsg << "DynamicGeometry: configuring\n";

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  // capture entity 
  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // connect to the SceneUpdate event
  // the callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
      _eventMgr.Connect<ignition::gazebo::events::SceneUpdate>(
      std::bind(&DynamicGeometryPrivate::OnUpdate, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void DynamicGeometry::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &)
{
  IGN_PROFILE("DynamicGeometry::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
DynamicGeometryPrivate::~DynamicGeometryPrivate()
{
  this->DestroyScene();
};

//////////////////////////////////////////////////
DynamicGeometryPrivate::DynamicGeometryPrivate()
{
  memset(mDynamicVertexBuffer, 0, sizeof(mDynamicVertexBuffer));
};

//////////////////////////////////////////////////
void DynamicGeometryPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->visualName.empty())
    return;

  if (!this->scene)
  {
    ignmsg << "DynamicGeometry: retrieving scene from render engine\n";
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene)
    return;

  if (!this->visual)
  {
    ignmsg << "DynamicGeometry: searching for visual\n";

    // this does a breadth first search for visual with the entity id
    // \todo(anyone) provide a helper function in RenderUtil to search for
    // visual by entity id?
    auto rootVis = scene->RootVisual();
    std::list<rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        // RenderUtil stores gazebo-entity user data as int
        // \todo(anyone) Change this to uint64_t in Ignition H?
        auto variant = n->UserData("gazebo-entity");
        const int *value = std::get_if<int>(&variant);
        if (value && *value == static_cast<int>(this->entity))
        {
          this->visual = std::dynamic_pointer_cast<rendering::Visual>(n);
          break;
        }
      }
      for (unsigned int i = 0; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }
  }

  if (!this->visual)
    return;

  // Test attaching an Ogre2 mesh to the entity
  if (!this->dynamicVisual)
  {
    ignmsg << "DynamicGeometry: creating Ogre2 Ocean Tile\n";

    // scene
    rendering::Ogre2ScenePtr ogre2Scene =
        std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);    

    auto meshFactory = rendering::Ogre2MeshFactoryExtPtr(
          new rendering::Ogre2MeshFactoryExt(ogre2Scene));

    rendering::MeshDescriptor meshDescriptor;
    meshDescriptor.mesh = nullptr;
    meshDescriptor.meshName = "dynamic-mesh";

    /////////////////////////////////////////////
    // BEGIN_LAMDAS
    //
    // Override functions from Scene, BaseScene, 
    // and Ogre2Scene
    //
    /////////////////////////////////////////////

    // bool Ogre2Scene::InitObject(Ogre2ObjectPtr _object, unsigned int _id,
    //     const std::string &_name)
    auto Ogre2Scene_InitObject = [&](
        rendering::Ogre2ScenePtr _scene,
        rendering::Ogre2ObjectPtr _object,
        unsigned int _id,
        const std::string &_name) -> bool
    {
      rendering::Ogre2MeshExtPtr derived =
          std::dynamic_pointer_cast<rendering::Ogre2MeshExt>(_object);

      if (!derived)
        return false;
      else
        derived->InitObject(_scene, _id, _name);
  
      return true;
    };

    // Ogre2Scene::CreateMeshImpl(unsigned int _id,
    //     const std::string &_name, const MeshDescriptor &_desc) -> rendering::MeshPtr 
    auto Ogre2Scene_CreateMeshImpl = [&](
        rendering::Ogre2ScenePtr _scene,
        unsigned int _id,
        const std::string &_name,
        const rendering::MeshDescriptor &_desc)
        -> rendering::MeshPtr
    {
      rendering::Ogre2MeshPtr mesh = meshFactory->Create(_desc);
      if (nullptr == mesh)
        return nullptr;
      mesh->SetDescriptor(_desc);

      bool result = Ogre2Scene_InitObject(_scene, mesh, _id, _name);
      return (result) ? mesh : nullptr;
    };

    // BaseScene::CreateMesh(const MeshDescriptor &_desc) -> rendering::MeshPtr 
    auto BaseScene_CreateMesh = [&](
        rendering::Ogre2ScenePtr _scene,
        const rendering::MeshDescriptor &_desc)
        -> rendering::MeshPtr 
    {
      std::string meshName = (_desc.mesh) ?
          _desc.mesh->Name() : _desc.meshName;

      // \todo: implement equivalents
      // unsigned int objId = this->CreateObjectId();
      // std::string objName = this->CreateObjectName(objId, "Mesh-" + meshName);
      unsigned int objId = 50000;
      std::string objName = "Mesh-" + meshName;
      return Ogre2Scene_CreateMeshImpl(_scene, objId, objName, _desc);
    };

    /////////////////////////////////////////////
    // END_LAMDAS
    /////////////////////////////////////////////

    // create the rendering mesh
    rendering::MeshPtr renderingMesh = BaseScene_CreateMesh(
        ogre2Scene, meshDescriptor);

    // attach mesh to visuals
    auto visual = this->scene->CreateVisual("dynamic-geometry");
    visual->AddGeometry(renderingMesh);
    visual->SetLocalPosition(0.0, 0.0, 0.0);
    visual->SetLocalRotation(0.0, 0.0, 0.0);
    visual->SetLocalScale(1.0, 1.0, 1.0);
    visual->SetMaterial("Default/White");
    visual->SetVisible(true);

    // add visual to parent
    auto parent = this->visual->Parent();
    parent->AddChild(visual);

    // keep reference
    this->dynamicVisual = visual;

    // Ogre2 Sample Dynamic Geometry
    this->CreateScene01();
  }

  if (!this->dynamicVisual)
    return;

  float simTime = (std::chrono::duration_cast<std::chrono::nanoseconds>(
      this->currentSimTime).count()) * 1e-9;

  this->Update(simTime);
}

//////////////////////////////////////////////////
Ogre::RenderSystem * DynamicGeometryPrivate::RenderSystem() const
{
  rendering::Ogre2ScenePtr ogreScene =
      std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);
  Ogre::SceneManager *sceneManager = ogreScene->OgreSceneManager();
  Ogre::RenderSystem *renderSystem = sceneManager->getDestinationRenderSystem();
  return renderSystem;
}

//////////////////////////////////////////////////
Ogre::IndexBufferPacked* DynamicGeometryPrivate::CreateIndexBuffer(void)
{
  Ogre::IndexBufferPacked *indexBuffer = 0;

  const Ogre::uint16 c_indexData[3 * 2 * 6] =
  {
      0, 1, 2, 2, 3, 0, //Front face
      6, 5, 4, 4, 7, 6, //Back face

      3, 2, 6, 6, 7, 3, //Top face
      5, 1, 0, 0, 4, 5, //Bottom face

      4, 0, 3, 3, 7, 4, //Left face
      6, 2, 1, 1, 5, 6, //Right face
  };

  Ogre::uint16 *cubeIndices = reinterpret_cast<Ogre::uint16*>(
      OGRE_MALLOC_SIMD(
          sizeof(Ogre::uint16) * 3 * 2 * 6,
          Ogre::MEMCATEGORY_GEOMETRY));
  memcpy(cubeIndices, c_indexData, sizeof(c_indexData));

  // Ogre::Root *root = mGraphicsSystem->getRoot();
  // Ogre::RenderSystem *renderSystem = root->getRenderSystem();
  Ogre::RenderSystem *renderSystem = this->RenderSystem();
  Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

  try
  {
    indexBuffer = vaoManager->createIndexBuffer(
        Ogre::IndexBufferPacked::IT_16BIT,
        3 * 2 * 6,
        Ogre::BT_IMMUTABLE,
        cubeIndices,
        true);
  }
  catch(Ogre::Exception &e)
  {
      // When keepAsShadow = true, the memory will be freed when the index buffer is destroyed.
      // However if for some weird reason there is an exception raised, the memory will
      // not be freed, so it is up to us to do so.
      // The reasons for exceptions are very rare. But we're doing this for correctness.
      // Important: Please note that we passed keepAsShadow = true to createIndexBuffer,
      // thus Ogre will free the pointer. However had we passed keepAsShadow = false,
      // it would be YOUR responsability to free the pointer, not Ogre's
      OGRE_FREE_SIMD(indexBuffer, Ogre::MEMCATEGORY_GEOMETRY);
      indexBuffer = 0;
      throw e;
  }

  return indexBuffer;
}

//////////////////////////////////////////////////
Ogre::MeshPtr DynamicGeometryPrivate::CreateStaticMesh(bool partialMesh)
{
  // Ogre::Root *root = mGraphicsSystem->getRoot();
  // Ogre::RenderSystem *renderSystem = root->getRenderSystem();
  Ogre::RenderSystem *renderSystem = this->RenderSystem();
  Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

  //Create the mesh
  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
              partialMesh ? "My PartialMesh" : "My StaticMesh",
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

  //Create one submesh
  Ogre::SubMesh *subMesh = mesh->createSubMesh();

  //Vertex declaration
  Ogre::VertexElement2Vec vertexElements;
  vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_POSITION ) );
  vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_NORMAL ) );

  //For immutable buffers, it is mandatory that cubeVertices is not a null pointer.
  CubeVertices *cubeVertices = reinterpret_cast<CubeVertices*>( OGRE_MALLOC_SIMD(
                                                                    sizeof(CubeVertices) * 8,
                                                                    Ogre::MEMCATEGORY_GEOMETRY ) );
  //Fill the data.
  memcpy( cubeVertices, c_originalVertices, sizeof(CubeVertices) * 8 );

  Ogre::VertexBufferPacked *vertexBuffer = 0;
  try
  {
      //Create the actual vertex buffer.
      vertexBuffer = vaoManager->createVertexBuffer( vertexElements, 8,
                                                      partialMesh ? Ogre::BT_DEFAULT :
                                                                    Ogre::BT_IMMUTABLE,
                                                      cubeVertices, true );
  }
  catch( Ogre::Exception &e )
  {
      // Important: Please note that we passed keepAsShadow = true to createVertexBuffer,
      // thus Ogre will free the pointer. However had we passed keepAsShadow = false,
      // it would be YOUR responsability to free the pointer, not Ogre's
      OGRE_FREE_SIMD( vertexBuffer, Ogre::MEMCATEGORY_GEOMETRY );
      vertexBuffer = 0;
      throw e;
  }

  //Now the Vao. We'll just use one vertex buffer source (multi-source not working yet)
  Ogre::VertexBufferPackedVec vertexBuffers;
  vertexBuffers.push_back( vertexBuffer );
  Ogre::IndexBufferPacked *indexBuffer = this->CreateIndexBuffer(); //Create the actual index buffer
  Ogre::VertexArrayObject *vao = vaoManager->createVertexArrayObject(
              vertexBuffers, indexBuffer, Ogre::OT_TRIANGLE_LIST );

  //Each Vao pushed to the vector refers to an LOD level.
  //Must be in sync with mesh->mLodValues & mesh->mNumLods if you use more than one level
  subMesh->mVao[Ogre::VpNormal].push_back( vao );
  //Use the same geometry for shadow casting.
  subMesh->mVao[Ogre::VpShadow].push_back( vao );

  //Set the bounds to get frustum culling and LOD to work correctly.
  mesh->_setBounds( Ogre::Aabb( Ogre::Vector3::ZERO, Ogre::Vector3::UNIT_SCALE ), false );
  mesh->_setBoundingSphereRadius( 1.732f );

  return mesh;
}

//////////////////////////////////////////////////
std::pair<Ogre::MeshPtr, Ogre::VertexBufferPacked*> DynamicGeometryPrivate::CreateDynamicMesh(size_t idx)
{
  // Ogre::Root *root = mGraphicsSystem->getRoot();
  // Ogre::RenderSystem *renderSystem = root->getRenderSystem();
  Ogre::RenderSystem *renderSystem = this->RenderSystem();
  Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
              "My DynamicMesh_" + Ogre::StringConverter::toString(idx),
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

  Ogre::SubMesh *subMesh = mesh->createSubMesh();

  Ogre::VertexElement2Vec vertexElements;
  vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_POSITION ) );
  vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_NORMAL ) );

  //Pass cubeVertices to have initialized values. However you can safely use a null pointer.
  CubeVertices cubeVertices[8];
  memcpy( cubeVertices, c_originalVertices, sizeof(CubeVertices) * 8 );

  Ogre::VertexBufferPacked *vertexBuffer = 0;
  vertexBuffer = vaoManager->createVertexBuffer( vertexElements, 8,
                                                  Ogre::BT_DYNAMIC_PERSISTENT,
                                                  cubeVertices, false );

  //Now the Vao
  Ogre::VertexBufferPackedVec vertexBuffers;
  vertexBuffers.push_back( vertexBuffer );
  Ogre::IndexBufferPacked *indexBuffer = this->CreateIndexBuffer();
  Ogre::VertexArrayObject *vao = vaoManager->createVertexArrayObject(
              vertexBuffers, indexBuffer, Ogre::OT_TRIANGLE_LIST );

  subMesh->mVao[Ogre::VpNormal].push_back( vao );
  //Use the same geometry for shadow casting.
  subMesh->mVao[Ogre::VpShadow].push_back( vao );

  //Set the bounds to get frustum culling and LOD to work correctly.
  mesh->_setBounds( Ogre::Aabb( Ogre::Vector3::ZERO, Ogre::Vector3::UNIT_SCALE ), false );
  mesh->_setBoundingSphereRadius( 1.732f );

  return std::pair<Ogre::MeshPtr, Ogre::VertexBufferPacked*>( mesh, vertexBuffer );
}

//////////////////////////////////////////////////
void DynamicGeometryPrivate::UpdateDynamicBuffer01(float *cubeVertices,
    const CubeVertices originalVerts[8],
    size_t start, size_t end) const
{
  const float cosAlpha = cosf( mRotationTime );
  const float sinAlpha = sinf( mRotationTime );

  for( size_t i=start; i<end; ++i )
  {
    cubeVertices[0] = originalVerts[i].px * cosAlpha - originalVerts[i].pz * sinAlpha;
    cubeVertices[1] = originalVerts[i].py;
    cubeVertices[2] = originalVerts[i].px * sinAlpha + originalVerts[i].pz * cosAlpha;

    cubeVertices[3] = originalVerts[i].nx * cosAlpha - originalVerts[i].nz * sinAlpha;
    cubeVertices[4] = originalVerts[i].ny;
    cubeVertices[5] = originalVerts[i].nx * sinAlpha + originalVerts[i].nz * cosAlpha;

    cubeVertices += 6;
  }
}

//////////////////////////////////////////////////
void DynamicGeometryPrivate::Update(float simTime)
{
  mRotationTime = simTime;
  // mRotationTime += timeSinceLast;
  mRotationTime = fmod( mRotationTime, Ogre::Math::PI * 2.0f );

  const float cosAlpha = cosf( mRotationTime );
  const float sinAlpha = sinf( mRotationTime );

  {
      //Partial update the buffer's 2nd vertex.
      Ogre::VertexBufferPacked *partialVertexBuffer = mPartialMesh->getSubMesh( 0 )->
              mVao[Ogre::VpNormal][0]->getVertexBuffers()[0];
      CubeVertices newVertex( c_originalVertices[2] );
      newVertex.px += cosAlpha;
      partialVertexBuffer->upload( &newVertex, 2, 1 );
  }


  //----------------------------------------------------------------
  //First dynamic buffer example.

  //Dynamic buffers assume you will be fully uploading the entire buffer's contents
  //every time you map them.
  //"Partially" mapping or filling the buffer will not result in desired results
  //(data uploaded in previous frames will get mixed with with the
  //new data you're uploading)

  //You should NEVER read from cubeVertices pointer. Beware that something as innocent as
  //++(*cubeVertices) or cubeVertices[0] += 1; or cubesVertices[1] = cubesVertices[0];
  //implies reading from the mapped memory.
  //
  //Reading from this memory may return garbage, may return old data (from previous frames)
  //and will probably be *very* slow since the memory region is often write combined.
  //Sometimes you need to check the assembly to see the compiler isn't reading from
  //that memory even though the C++ code doesn't.
  float * RESTRICT_ALIAS cubeVertices = reinterpret_cast<float*RESTRICT_ALIAS>(
              mDynamicVertexBuffer[0]->map( 0, mDynamicVertexBuffer[0]->getNumElements() ) );

  for( size_t i=0; i<8; ++i )
  {
      cubeVertices[0] = c_originalVertices[i].px * cosAlpha - c_originalVertices[i].py * sinAlpha;
      cubeVertices[1] = c_originalVertices[i].px * sinAlpha + c_originalVertices[i].py * cosAlpha;
      cubeVertices[2] = c_originalVertices[i].pz;

      cubeVertices[3] = c_originalVertices[i].nx * cosAlpha - c_originalVertices[i].ny * sinAlpha;
      cubeVertices[4] = c_originalVertices[i].nx * sinAlpha + c_originalVertices[i].ny * cosAlpha;
      cubeVertices[5] = c_originalVertices[i].nz;

      cubeVertices += 6;
  }

  mDynamicVertexBuffer[0]->unmap( Ogre::UO_KEEP_PERSISTENT );

  //----------------------------------------------------------------
  //Second dynamic buffer example.
  //  Update the cube mapping multiple times per frame.
  //  Every time you map, you 'advance' the buffer for the next frame.
  //  If you want to map it again within the same frame, you first
  //  need to 'regress' the buffer back to its original state,
  //  while being carefull that:
  //      1. Once you're done with all your maps, and by the time rendering starts,
  //         the buffer has advanced ONLY once.
  //      2. You do not write to a memory region that you have already written to,
  //         unless you know you haven't issued draw calls that are using this region yet.

  //The last 'false' indicates the buffer not to advance forward.
  cubeVertices = reinterpret_cast<float*RESTRICT_ALIAS>(
              mDynamicVertexBuffer[1]->map( 0, 2, false ) );
  this->UpdateDynamicBuffer01( cubeVertices, c_originalVertices, 0, 2 );
  mDynamicVertexBuffer[1]->unmap( Ogre::UO_KEEP_PERSISTENT );

  //We do not regress the frame, because we haven't advanced yet.
  cubeVertices = reinterpret_cast<float*RESTRICT_ALIAS>(
              mDynamicVertexBuffer[1]->map( 2, 2, true ) );
  this->UpdateDynamicBuffer01( cubeVertices, c_originalVertices, 2, 4 );
  mDynamicVertexBuffer[1]->unmap( Ogre::UO_KEEP_PERSISTENT );

  //We regress the frame, because the previous map had advanced automatically by passing 'true'.
  mDynamicVertexBuffer[1]->regressFrame();
  cubeVertices = reinterpret_cast<float*RESTRICT_ALIAS>(
              mDynamicVertexBuffer[1]->map( 4, 2, false ) );
  this->UpdateDynamicBuffer01( cubeVertices, c_originalVertices, 4, 6 );
  mDynamicVertexBuffer[1]->unmap( Ogre::UO_KEEP_PERSISTENT );
  mDynamicVertexBuffer[1]->advanceFrame();    //Make sure we advance when we're done and
                                              //draw calls may start afterwards.

  // ... hypothetical draw calls issued ...

  //We regress the frame, because the previous map had advanced it; and the frame isn't over yet.
  mDynamicVertexBuffer[1]->regressFrame();
  cubeVertices = reinterpret_cast<float*RESTRICT_ALIAS>(
              mDynamicVertexBuffer[1]->map( 6, 2, false ) );
  this->UpdateDynamicBuffer01( cubeVertices, c_originalVertices, 6, 8 );
  mDynamicVertexBuffer[1]->unmap( Ogre::UO_KEEP_PERSISTENT );
  mDynamicVertexBuffer[1]->advanceFrame();    //Make sure we advance when we're done and
                                              //draw calls may start afterwards.

  // TutorialGameState::update( timeSinceLast );
}

//////////////////////////////////////////////////
void DynamicGeometryPrivate::CreateScene01(void)
{
  //Create all four types of meshes.
  mStaticMesh  = CreateStaticMesh( false );
  mPartialMesh = CreateStaticMesh( true );

  for( size_t i=0; i<2; ++i )
  {
      std::pair<Ogre::MeshPtr, Ogre::VertexBufferPacked*> dynamicMesh;
      dynamicMesh = CreateDynamicMesh( i );
      mDynamicMesh[i]         = dynamicMesh.first;
      mDynamicVertexBuffer[i] = dynamicMesh.second;
  }

  //Initialize the scene (items)
  rendering::Ogre2ScenePtr ogreScene =
      std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);
  Ogre::SceneManager *sceneManager = ogreScene->OgreSceneManager();
  // Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();

  Ogre::Item *item = sceneManager->createItem( mStaticMesh, Ogre::SCENE_DYNAMIC );
  Ogre::SceneNode *sceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
          createChildSceneNode( Ogre::SCENE_DYNAMIC );
  sceneNode->attachObject( item );
  sceneNode->setPosition( -6, 0, 0 );

  item = sceneManager->createItem( mPartialMesh, Ogre::SCENE_DYNAMIC );
  sceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
          createChildSceneNode( Ogre::SCENE_DYNAMIC );
  sceneNode->attachObject( item );
  sceneNode->setPosition( -2, 0, 0 );

  item = sceneManager->createItem( mDynamicMesh[0], Ogre::SCENE_DYNAMIC );
  sceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
          createChildSceneNode( Ogre::SCENE_DYNAMIC );
  sceneNode->attachObject( item );
  sceneNode->setPosition( 2, 0, 0 );

  item = sceneManager->createItem( mDynamicMesh[1], Ogre::SCENE_DYNAMIC );
  sceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
          createChildSceneNode( Ogre::SCENE_DYNAMIC );
  sceneNode->attachObject( item );
  sceneNode->setPosition( 6, 0, 0 );

  // Ogre::Light *light = sceneManager->createLight();
  // Ogre::SceneNode *lightNode = sceneManager->getRootSceneNode()->createChildSceneNode();
  // lightNode->attachObject( light );
  // light->setPowerScale( Ogre::Math::PI ); //Since we don't do HDR, counter the PBS' division by PI
  // light->setType( Ogre::Light::LT_DIRECTIONAL );
  // light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );

  // mCameraController = new CameraController( mGraphicsSystem, false );
}

//////////////////////////////////////////////////
void DynamicGeometryPrivate::DestroyScene()
{
  for(size_t i=0; i<2; ++i)
  {
    //Permanently unmap persistent mapped buffers
    if( mDynamicVertexBuffer[i] &&
        mDynamicVertexBuffer[i]->getMappingState() != Ogre::MS_UNMAPPED )
    {
        mDynamicVertexBuffer[i]->unmap( Ogre::UO_UNMAP_ALL );
    }
  }

  //If we don't do this, the smart pointers will try to
  //delete memory after Ogre has shutdown (and crash)
  mStaticMesh.setNull();
  mPartialMesh.setNull();
  for( size_t i=0; i<2; ++i )
    mDynamicMesh[i].setNull();
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(DynamicGeometry,
                    ignition::gazebo::System,
                    DynamicGeometry::ISystemConfigure,
                    DynamicGeometry::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DynamicGeometry,
  "ignition::gazebo::systems::DynamicGeometry")
