
#include "ignition/rendering/ogre2/Ogre2OceanVisual.hh"

#include <ignition/rendering/ogre2/Ogre2DynamicRenderable.hh>
#include <ignition/rendering/ogre2/Ogre2Material.hh>

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneNode.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

class ignition::rendering::Ogre2OceanVisualPrivate
{
  /// \brief Grid materal
  public: Ogre2MaterialPtr material = nullptr;

  /// \brief Lines that make the cross marking the center of mass.
  public: std::shared_ptr<Ogre2DynamicRenderable> crossLines = nullptr;

  /// \brief Sphere visual marking the center of mass
  public: VisualPtr sphereVis = nullptr;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2OceanVisual::Ogre2OceanVisual()
  : dataPtr(new Ogre2OceanVisualPrivate)
{
}

//////////////////////////////////////////////////
Ogre2OceanVisual::~Ogre2OceanVisual()
{
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::PreRender()
{
  if (this->HasParent() && this->parentName.empty())
    this->parentName = this->Parent()->Name();

  if (this->dirtyOceanVisual &&
      !this->parentName.empty())
  {
    this->parentName = this->Parent()->Name();
    this->CreateVisual();
    this->dirtyOceanVisual = false;
  }
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::Init()
{
  BaseOceanVisual::Init();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::Destroy()
{
  if (this->dataPtr->sphereVis != nullptr)
  {
    this->dataPtr->sphereVis->Destroy();
    this->dataPtr->sphereVis.reset();
  }

  if (this->dataPtr->crossLines)
  {
    this->dataPtr->crossLines->Destroy();
    this->dataPtr->crossLines.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::CreateVisual()
{
  if (!this->dataPtr->crossLines)
  {
    this->dataPtr->crossLines.reset(
        new Ogre2DynamicRenderable(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->crossLines->OgreObject());
  }

  if (!this->dataPtr->sphereVis)
  {
    this->dataPtr->sphereVis = this->Scene()->CreateVisual();
    this->dataPtr->sphereVis->AddGeometry(this->Scene()->CreateSphere());
    this->dataPtr->sphereVis->SetMaterial("Default/CoM");
    this->dataPtr->sphereVis->SetInheritScale(false);
    this->AddChild(this->dataPtr->sphereVis);
  }

  double sphereRadius = this->SphereRadius();
  this->dataPtr->sphereVis->SetLocalScale(ignition::math::Vector3d(
      sphereRadius*2, sphereRadius*2, sphereRadius*2));
  this->dataPtr->sphereVis->SetLocalPosition(this->InertiaPose().Pos());
  this->dataPtr->sphereVis->SetLocalRotation(this->InertiaPose().Rot());

  // Get the bounding box of the parent visual
  VisualPtr vis = this->Scene()->VisualByName(this->parentName);
  ignition::math::AxisAlignedBox box;
  if (vis)
    box = vis->LocalBoundingBox();

  // Clear any previous data from the grid and update
  this->dataPtr->crossLines->Clear();
  this->dataPtr->crossLines->Update();

  this->dataPtr->crossLines->SetOperationType(MarkerType::MT_LINE_LIST);
  if (!this->dataPtr->material)
  {
    MaterialPtr OceanVisualMaterial =
        this->Scene()->Material("Default/TransGreen")->Clone();
    this->SetMaterial(OceanVisualMaterial, false);
  }

  // CoM position indicator
  ignition::math::Vector3d p1(0, 0,
      box.Min().Z() - this->InertiaPose().Pos().Z());
  ignition::math::Vector3d p2(0, 0,
      box.Max().Z() - this->InertiaPose().Pos().Z());

  ignition::math::Vector3d p3(0,
      box.Min().Y() - this->InertiaPose().Pos().Y(), 0);
  ignition::math::Vector3d p4(0,
      box.Max().Y() - this->InertiaPose().Pos().Y(), 0);

  ignition::math::Vector3d p5(
      box.Min().X() - this->InertiaPose().Pos().X(), 0, 0);
  ignition::math::Vector3d p6(
      box.Max().X() - this->InertiaPose().Pos().X(), 0, 0);

  p1 += this->InertiaPose().Pos();
  p2 += this->InertiaPose().Pos();
  p3 += this->InertiaPose().Pos();
  p4 += this->InertiaPose().Pos();
  p5 += this->InertiaPose().Pos();
  p6 += this->InertiaPose().Pos();

  this->dataPtr->crossLines->AddPoint(p1);
  this->dataPtr->crossLines->AddPoint(p2);
  this->dataPtr->crossLines->AddPoint(p3);
  this->dataPtr->crossLines->AddPoint(p4);
  this->dataPtr->crossLines->AddPoint(p5);
  this->dataPtr->crossLines->AddPoint(p6);

  this->dataPtr->crossLines->Update();
  this->ogreNode->setVisible(true);
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  Ogre2MaterialPtr derived =
      std::dynamic_pointer_cast<Ogre2Material>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  // Set material for the underlying dynamic renderable
  this->dataPtr->crossLines->SetMaterial(_material, false);
  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::SetMaterialImpl(Ogre2MaterialPtr _material)
{
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->material = _material;
}

//////////////////////////////////////////////////
MaterialPtr Ogre2OceanVisual::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
VisualPtr Ogre2OceanVisual::SphereVisual() const
{
  return this->dataPtr->sphereVis;
}
