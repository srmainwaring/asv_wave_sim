// Copyright (C) 2019  Rhys Mainwaring
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

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SPECTRUM_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SPECTRUM_HH_

namespace asv
{

  class WaveSpectrum
  {
    public: virtual ~WaveSpectrum();

    public: WaveSpectrum();

    public: void SetWindVelocity(double _ux, double _uy);
    
    public: static double Dispersion(double _k);

    public: static double InvDispersion(double _omega);

    public: static double QuantisedDispersion(double _k);

    public: static double SignificantWaveHeight(double _u);

    public: static double Spectrum(double _k, double _kx, double _ky, double _u, double _ux, double _uy);

    public: static double PiersonMoskowitzK0(double _u);

    public: static double PiersonMoskowitzSpectrum(double _k, double _kx, double _ky, double _u, double _ux, double _uy);

    private: double ux;
    private: double uy;
    private: double u;
  };

}

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SPECTRUM_HH_
