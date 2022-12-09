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

#ifndef GZ_RENDERING_BASE_RENDERENGINEEXTENSION_HH_
#define GZ_RENDERING_BASE_RENDERENGINEEXTENSION_HH_

#include <map>
#include <string>

#include <gz/rendering/config.hh>
#include <gz/rendering/GraphicsAPI.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Export.hh>

#include "RenderEngineExtension.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

class GZ_RENDERING_VISIBLE BaseRenderEngineExtension :
    public virtual RenderEngineExtension
{
  protected: BaseRenderEngineExtension();

  public: virtual ~BaseRenderEngineExtension();

  public: virtual bool Load(
      const std::map<std::string, std::string> &_params = {}) override;

  public: virtual bool Init() override;

  public: virtual void Destroy() override;

  public: virtual bool IsInitialized() const override;

  protected: virtual bool LoadImpl(
      const std::map<std::string, std::string> &_params) = 0;

  protected: virtual bool InitImpl() = 0;

  protected: bool loaded = false;

  protected: bool initialized = false;
};

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_BASE_RENDERENGINEEXTENSION_HH_
