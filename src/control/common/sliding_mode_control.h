
namespace fairpsace {
namespace control {

class SMSuperTwistSolver 
{
 public:
  SMSuperTwistSolver(Config);
  double compute(double error);
}

} // control  
} // fairspace