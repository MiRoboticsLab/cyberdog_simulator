// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _ACTUATOR_HPP__
#define _ACTUATOR_HPP__

#include <math.h>
#include <stdio.h>

#include "ctrl_ros/utilities/utilities.h"




namespace gazebo
{
  class Actuator
  {
  public:
    /**
     * @brief Construct a new Actuator object
     * 
     * @param n1  : velocity of TN curve
     * @param n2  : velocity of TN curve
     * @param n3  : velocity of TN curve
     * @param tau_max : maximum torque
     * @param it_curve_a   : coefficient of I-tau curve
     * @param it_curve_b   : coefficient of I-tau curve
     * @param it_curve_c   : coefficient of I-tau curve
     */
    Actuator(const double &n1 = 11.52, const double &n2 = 29.32, const double &n3 = 60.0, const double &tau_max = 12.001,  
             const double &it_curve_a = 0.0022, const double &it_curve_b = -0.0067, const double &it_curve_c = 1.1246)
    {
      n1_ = n1;     
      n2_ = n2;     
      n3_ = n3;
      tau_max_ = tau_max;    
      it_curve_a_ = it_curve_a;     
      it_curve_b_ = it_curve_b;     
      it_curve_c_ = it_curve_c; 
    }

    ~Actuator(){};

    /*!
    * Compute actual actuator torque, given desired torque and speed.
    * takes into account friction (dry and damping), voltage limits, and torque
    * limits
    * @param tauDes : desired torque
    * @param qd : current actuator velocity (at the joint)
    * @return actual produced torque
    */
    double GetTorque(double tau_des, double qd){
      double tau_act_ = tau_des;

      // TN curve
        if ( qd < -n3_ ) {
            tau_act_ = 0;
            printf( "actuator speed less than min\n" );
        }

        if ( qd >= -n3_ && qd < -n2_ ) {
            if ( tau_act_ > tau_max_ ) {
                tau_act_ = tau_max_;
            }
            if ( tau_act_ < 0 ) {
                tau_act_ = 0;
            }
        }

        if ( qd >= -n2_ && qd < -n1_ ) {
            double tau_act_min = ( tau_max_ ) / ( n1_ - n2_ ) * qd - ( -( ( tau_max_ ) * n2_ ) / ( n1_ - n2_ ) );
            if ( tau_act_ > tau_max_ ) {
                tau_act_ = tau_max_;
            }
            if ( tau_act_ < tau_act_min ) {
                tau_act_ = tau_act_min;
            }
        }

        if ( qd >= -n1_ && qd < n1_ ) {
            if ( tau_act_ > tau_max_ ) {
                tau_act_ = tau_max_;
            }
            if ( tau_act_ < -tau_max_ ) {
                tau_act_ = -tau_max_;
            }
        }

        if ( qd >= n1_ && qd < n2_ ) {
            double tau_act_max = ( tau_max_ ) / ( n1_ - n2_ ) * qd + ( -( ( tau_max_ ) * n2_ ) / ( n1_ - n2_ ) );
            if ( tau_act_ > tau_act_max ) {
                tau_act_ = tau_act_max;
            }
            if ( tau_act_ < -tau_max_ ) {
                tau_act_ = -tau_max_;
            }
        }

        if ( qd >= n2_ && qd < n3_ ) {
            if ( tau_act_ < -tau_max_ ) {
                tau_act_ = -tau_max_;
            }
            if ( tau_act_ > 0 ) {
                tau_act_ = 0;
            }
        }

        if ( qd >= n3_ ) {
            tau_act_ = 0;
            printf( "actuator speed over max\n" );
        }

      return tau_act_;
    }

    double CerrentLoopResponse(double tauDes,double qd, int i){
      if(!first_in_[i]) {
        first_in_[i] = true;
        double I_motor = getActuatorI(tauDes, qd);
        i_last_[i] = I_motor;
        tau_last_[i] = tauDes;
        
        return tauDes;
      }
      else{
        double I_motor = getActuatorI(tauDes, qd);
        double d_I = I_motor-i_last_[i];

        if(d_I < 3.0 && d_I > -3.0){
          i_last_[i] = I_motor;
          tau_last_[i] = tauDes;
          return tauDes;
        }
        
        if(d_I > 3.0)
        {
          d_I = 3.0;
        }
        if(d_I < -3.0)
        {
          d_I = -3.0;
        }
        I_motor = i_last_[i]+d_I; 
        i_last_[i] = I_motor;
        tauDes = getActuatorT(I_motor, tau_last_[i]);
        tau_last_[i] = tauDes;
        return tauDes;
      }
    }

    /**
     * @brief Get the Actuator current
     * 
     * @param tauDes : desired torque
     * @param qd     : speed of motor
     * @return double 
     */
    double getActuatorI(double tauDes, double qd){
      
      double I_motor;
      I_motor = it_curve_a_ * tauDes * tauDes * tauDes + it_curve_b_ * tauDes * tauDes + it_curve_c_ * tauDes;
      
      return I_motor;
    }

    /**
     * @brief Get the Actuator Torque
     * 
     * @param I_motor  : motor current
     * @param tau_last_ : torque at last step
     * @return double 
     */
    double getActuatorT(double I_motor, double tau_last_){      
      double k_tau = 3*it_curve_a_*tau_last_*tau_last_+2*it_curve_b_*tau_last_+it_curve_c_;
      tau_last_ = tau_last_ - (it_curve_a_ * tau_last_ * tau_last_ * tau_last_ + it_curve_b_ * tau_last_ * tau_last_ + it_curve_c_ * tau_last_-I_motor)/k_tau;
      while(fabs(it_curve_a_ * tau_last_ * tau_last_ * tau_last_ + it_curve_b_ * tau_last_ * tau_last_ + it_curve_c_ * tau_last_-I_motor)>1e-5) {
          k_tau = 3*it_curve_a_*tau_last_*tau_last_+2*it_curve_b_*tau_last_+it_curve_c_;
          tau_last_ = tau_last_ - (it_curve_a_ * tau_last_ * tau_last_ * tau_last_ + it_curve_b_ * tau_last_ * tau_last_ + it_curve_c_ * tau_last_-I_motor)/k_tau;
      }
              
      return tau_last_;
    }



  private:
    double tau_max_;      // maximum torque
    double n1_;     // velocity of TN curve
    double n2_;     // velocity of TN curve
    double n3_;     // velocity of TN curve
    double it_curve_a_;      // coefficient of I-tau curve
    double it_curve_b_;      // coefficient of I-tau curve
    double it_curve_c_;      // coefficient of I-tau curve
    bool first_in_[12];
    double i_last_[12];
    double tau_last_[12];
    
  };

}

#endif //_ACTUATOR_HPP__