within Buildings.Applications.DHC.Loads;
model Ideal_CoolingLoad_Yanfei
 //Modelica.Blocks.Interfaces.RealInput Q_dot(unit="W") "heat flux, or cooling load, in W";
 //Modelica.Blocks.Interfaces.RealInput m_dot(unit="kg/s") "nominal flow rate, kg/s";

 Modelica.Blocks.Interfaces.RealOutput T_supply(unit="C")
    "supply air temperature";

 parameter Real T_return=7 "return water tempeature";
 parameter Real Cp=4.2 "water specific heat, J/Kg-K";

package Medium = Buildings.Media.Water "Medium model";

parameter Real h_outflow_start=1.0;
parameter Integer energyDynamics=2;
parameter Integer massDynamics=2;
parameter Real p_start= 1.0;
parameter Real X_start= 1.0;
parameter Real T_start= 20.0;
parameter Real C_start= 20.0;
parameter Boolean allowFlowReversal=false;
parameter Real m_flow_nominal= 20.0;
parameter Real tau= 1.0;
parameter Real rho_default= 1.0;
parameter Boolean from_dp= false;
parameter Real dp_nominal = 1.0;
parameter Boolean homotopyInitialization=false;
parameter Boolean linearizeFlowResistance=false;
parameter Real deltaM=10;
parameter Real m_dot=1.0;

  Fluid.HeatExchangers.HeaterCooler_u2 InternalGains(
    m_flow_nominal=1,
    dp_nominal=10,
    Q_flow_nominal=100)
    annotation (Placement(transformation(extent={{-108,-46},{106,100}})));
equation
  //Q_flow = Cp * m_dot * (T_supply - T_return);

    annotation (Placement(transformation(extent={{100,20},{120,40}})),
                 Placement(transformation(extent={{100,50},{120,70}})),
               Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-34,34},{32,-18}},
          lineColor={28,108,200},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}),                       Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Ideal_CoolingLoad_Yanfei;
