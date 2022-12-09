// Copyright (C) 2022  Rhys Mainwaring
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

#ifndef GZ_RENDERING_OGRE2_RENDERENGINEEXTENSION_HH_
#define GZ_RENDERING_OGRE2_RENDERENGINEEXTENSION_HH_

#include <map>
#include <memory>
#include <string>

#include <gz/common/SingletonT.hh>

#include <gz/rendering/config.hh>
#include <gz/rendering/Export.hh>

#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include <gz/rendering/ogre2/Export.hh>

#include "BaseRenderEngineExtension.hh"
#include "RenderEngineExtensionPlugin.hh"
#include "RenderEngineExtension.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

// Forward declaration
class  Ogre2RenderEngineExtensionPrivate;

class GZ_RENDERING_OGRE2_VISIBLE Ogre2RenderEngineExtensionPlugin :
  public RenderEngineExtensionPlugin
{
  public: Ogre2RenderEngineExtensionPlugin();

  public: ~Ogre2RenderEngineExtensionPlugin();

  public: std::string Name() const;

  public: RenderEngineExtension *Extension() const;
};

class GZ_RENDERING_OGRE2_VISIBLE Ogre2RenderEngineExtension :
    public virtual BaseRenderEngineExtension,
    public common::SingletonT<Ogre2RenderEngineExtension>
{
  private: Ogre2RenderEngineExtension();

  public: virtual ~Ogre2RenderEngineExtension();

  public: virtual void Destroy() override;

  public: virtual std::string Name() const override;

  public: virtual SceneNodeFactoryPtr SceneNodeFactory() const override;

  protected: virtual bool LoadImpl(
      const std::map<std::string, std::string> &_params) override;

  protected: virtual bool InitImpl() override;

  private: void LoadAttempt();

  private: void InitAttempt();

  /// \brief Pointer to private data
  private: std::unique_ptr<Ogre2RenderEngineExtensionPrivate> dataPtr;

  /// \brief Singleton setup
  private: friend class common::SingletonT<Ogre2RenderEngineExtension>;
};

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_OGRE2_RENDERENGINEEXTENSION_HH_
