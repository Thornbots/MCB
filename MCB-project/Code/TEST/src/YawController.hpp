/*
* The purpose of this class is to allow easy changing or tweaking of the controller that controls the yaw motor
* All of the constants that are used for the controller shall be placed inside of this class that ultimely only implements one function.
* 
* The function implemented in this class should take quite a few inputs (i.e. going to take: angular velocity of DT and T, angular acc of DT and T, and a number of other things)
* With this information, the function should calculate the voltage level to set the yaw motor at, check that it is within valid bounds, and return that value as a double.
*/

namespace ThornBots {
    class YawController {
        public: //Public Variables
        //System Constraints
            constexpr static double c = 0.0169; //[ks*s/m^2]
            constexpr static double J = 0.0289; //[kg*m^2]
            constexpr static double uk = 0.07; //[N/m]
            constexpr static double kb = 0.716; //[V*rad/s]
            constexpr static double kt = 0.741; //[N*m/A]
            constexpr static double ra = 8.705; //[ohm]
            constexpr static double gearRatio = 1; //[unitless]
            constexpr static double latency = 0.014; //[s]
        //Controller Constants
            constexpr static double kp = 11.1; //[s^-1]
            constexpr static double voltMax = 22.2; //[V]
            constexpr static double veloMax = voltMax / (kb * gearRatio); //[rad/s]
            constexpr static double ascale = 0.8; //[unitless]
            constexpr static double ks = uk * ra / (kt * gearRatio); //[A]
            constexpr static double kv = kb * gearRatio; //[V*s/rad]
            constexpr static double ka = J / (kt * gearRatio); //[A*s^2/rad]
            constexpr static double kvis = c / (kt * gearRatio); //[A*s/rad]
            constexpr static double kdt = -0.47; //[unitless]
            constexpr static double kdtrev = -0.7; //[unitless]
            constexpr static double kpv = 1.0; //[A*s/rad]
            constexpr static double kiv = 15.0; //[A/rad]
            constexpr static double ivmax = 0.1; //[funny units]
            constexpr static double intthresh = voltMax * 0.85; //[V]
            constexpr static double takeback = 0.05; //[unitless]

        private: //Private Variables

        public: //Public Methods
        double getDesiredVoltage(double driveTrainAngularVelocity, double driveTrainAngleRelativeToWorld, double turretAngularVelocity, double turretAngleRelativeToWorld, double ) {

        }

        private: //Private Methods

    };
}