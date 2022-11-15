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

#ifndef GZ_WAVE_SPREADING_FUNCTION_HH_
#define GZ_WAVE_SPREADING_FUNCTION_HH_

#include <Eigen/Dense>
 
using Eigen::MatrixXd;
 
namespace gz
{
namespace waves
{
inline namespace v2
{
  class DirectionalSpreadingFunction
  {
    public:
      virtual ~DirectionalSpreadingFunction();

      virtual double Evaluate(
          double _theta, double _theta_mean, double _k=1.0) const = 0;

      virtual void Evaluate(
          Eigen::Ref<Eigen::MatrixXd> _phi,
          const Eigen::Ref<const Eigen::MatrixXd> &_theta,
          double _theta_mean,
          const Eigen::Ref<const Eigen::MatrixXd> &_k=Eigen::MatrixXd())
          const = 0;
  };

  class Cos2sSpreadingFunction : public DirectionalSpreadingFunction
  {
    public:
      virtual ~Cos2sSpreadingFunction();

      Cos2sSpreadingFunction(double _spread=10.0);

      virtual double Evaluate(
          double _theta, double _theta_mean, double _k=1.0) const override;

      virtual void Evaluate(
          Eigen::Ref<Eigen::MatrixXd> _phi,
          const Eigen::Ref<const Eigen::MatrixXd> &_theta,
          double _theta_mean,
          const Eigen::Ref<const Eigen::MatrixXd> &_k=Eigen::MatrixXd())
          const override;

      double Spread() const;

      void SetSpread(double _value);

    private:
      void _RecalcCoeffs();

      double _spread{10.0};
      double _cap_c_s{0.0};
  };

  class ECKVSpreadingFunction : public DirectionalSpreadingFunction
  {
    public:
      virtual ~ECKVSpreadingFunction();

      ECKVSpreadingFunction(
          double _u10=5.0,
          double _cap_omega_c=0.84,
          double _gravity=9.81);

      virtual double Evaluate(
          double _theta, double _theta_mean, double _k=1.0) const override;

      virtual void Evaluate(
          Eigen::Ref<Eigen::MatrixXd> _phi,
          const Eigen::Ref<const Eigen::MatrixXd> &_theta,
          double _theta_mean,
          const Eigen::Ref<const Eigen::MatrixXd> &_k=Eigen::MatrixXd())
          const override;

      double Gravity() const;

      void SetGravity(double _value);

      double U10() const;

      void SetU10(double _value);

      double CapOmegaC() const;

      void SetCapOmegaC(double _value);

    private:
      double _gravity{9.81};
      double _u10{5.0};
      double _cap_omega_c{0.84};
  };
}
}
}

#endif
