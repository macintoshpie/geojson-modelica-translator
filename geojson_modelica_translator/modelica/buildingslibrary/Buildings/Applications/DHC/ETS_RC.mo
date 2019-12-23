within Buildings.Applications.DHC;
package ETS_RC "The connection of ETS and RC-building"
  extends EnergyTransferStations;
  model ETS_RC
    extends Buildings;

    annotation (__Dymola_DymolaStoredErrors(thetext="model ETS_RC
  extends Buildings;
    \"Example illustrating the coupling of a RC building model to a fluid loop\"
  import Buildings;
  extends Modelica.Icons.Example;
  //extends
  //extends Buildings.Applications.DHC.Loads.Examples.BaseClasses.RCBuilding;
 // extends Buildings.ThermalZones.ReducedOrder.RC.OneElement;

  package Medium = Buildings.Media.Water;

  parameter Modelica.SIunits.MassFlowRate mDis_flow_nominal = 0.5
    \"Nominal mass flow rate on district-side (primary)\";
  parameter Modelica.SIunits.MassFlowRate mBui_flow_nominal = 0.5
    \"Nominal mass flow rate on building-side (secondary)\";

  parameter Modelica.SIunits.SpecificHeatCapacity cp=
    Medium.specificHeatCapacityCp(
    Medium.setState_pTX(Medium.p_default, Medium.T_default, Medium.X_default))
    \"Specific heat capacity of medium\";

  parameter Real m1_flow_nominal=1.0;


  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
        \"modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos\"))
    \"Weather data reader\"
    annotation (Placement(transformation(extent={{78,10},{58,30}})));


  Buildings.Applications.DHC.Loads.Examples.BaseClasses.RCBuilding building
    annotation (Placement(transformation(extent={{74,-40},{94,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    nPorts=1,
    use_T_in=true)
              \"Supply for heating water\"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,58})));
  Buildings.Fluid.Sources.Boundary_pT sinHea(
    redeclare package Medium = Medium,
    nPorts=1,
    p=300000,
    T=couHea.T1_b_nominal)
              \"Sink for heating water\"annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={140,56})));
  Buildings.Applications.DHC.Loads.BaseClasses.HeatingOrCooling couHea(
    redeclare package Medium = Medium,
    flowRegime=building.floRegHeaLoa,
    T1_a_nominal=318.15,
    T1_b_nominal=313.15,
    Q_flow_nominal=building.Q_flowHea_nominal,
    T2_nominal=building.THeaLoa_nominal,
    m_flow2_nominal=building.m_flowHeaLoa_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    nLoa=building.nHeaLoa)
    annotation (Placement(transformation(extent={{30,66},{50,46}})));
  Modelica.Blocks.Sources.RealExpression m_flowHeaVal(y=couHea.m_flowReq)
    annotation (Placement(transformation(extent={{-96,58},{-76,78}})));
  Modelica.Blocks.Sources.RealExpression THeaInlVal(y=couHea.T1_a_nominal)
    annotation (Placement(transformation(extent={{-98,42},{-78,62}})));

  Buildings.Applications.DHC.EnergyTransferStations.CoolingIndirect coo
    annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  Modelica.Blocks.Sources.Constant TSetCHWS(k=273.15 + 7)
    \"Setpoint temperature for building chilled water supply\"
    annotation (Placement(transformation(extent={{-98,-52},{-78,-32}})));
  Modelica.Blocks.Sources.Trapezoid tra(
    amplitude=1.5,
    rising(displayUnit=\"h\") = 10800,
    width(displayUnit=\"h\") = 10800,
    falling(displayUnit=\"h\") = 10800,
    period(displayUnit=\"h\") = 43200,
    offset=273 + 3.5)
    \"District supply temperature trapezoid signal\"
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
  Buildings.Fluid.Sources.Boundary_pT souDis(
    redeclare package Medium = Medium,
    p(displayUnit=\"Pa\") = 300000 + 800,
    use_T_in=true,
    T=278.15,
    nPorts=1)
    \"District (primary) source\"
    annotation (Placement(transformation(extent={{-60,16},{-40,36}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort TDisSup(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    T_start=278.15)
    \"District-side (primary) supply temperature sensor\"
    annotation (Placement(transformation(extent={{-38,0},{-18,20}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort TDisRet(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    T_start=287.15)
    \"District-side (primary) return temperature sensor\"
    annotation (Placement(transformation(extent={{18,-24},{38,-4}})));
  Buildings.Fluid.Sources.Boundary_pT sinDis(
    redeclare package Medium = Medium,
    p=300000,
    T=287.15,
    nPorts=1)
    \"District-side (primary) sink\"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={16,24})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pumBui(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    nominalValuesDefineDefaultPressureCurve=true,
    dp_nominal=6000,
    m_flow_nominal=0.5)
    \"Building-side (secondary) pump\"
    annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-30,-106})));
  Buildings.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium,
      V_start=1000)
    \"Expansion tank\"
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={-76,-80})));
  Buildings.Fluid.HeatExchangers.HeaterCooler_u2 hea(
    m_flow_nominal=0.5,
    dp_nominal=2,
    Q_flow_nominal=100,
    redeclare package Medium = Buildings.Media.Water)
    annotation (Placement(transformation(extent={{52,-104},{72,-84}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowCooAct1[1]
    \"Actual cooling heat flow rate\"
    annotation (Placement(transformation(extent={{134,-88},{174,-48}})));
equation 
  connect(weaDat.weaBus, building.weaBus) annotation (Line(
      points={{58,20},{84,20},{84,-20},{84.1,-20}},
      color={255,204,51},
      thickness=0.5));
  connect(supHea.ports[1], couHea.port_a)
  annotation (Line(points={{-20,58},{-12,58},{-12,56},{30,56}},   color={0,127,255}));
  connect(couHea.port_b, sinHea.ports[1])
  annotation (Line(points={{50,56},{130,56}},                   color={0,127,255}));
  connect(supHea.m_flow_in, m_flowHeaVal.y)
    annotation (Line(points={{-42,66},{-66,66},{-66,68},{-75,68}}, color={0,0,127}));
  connect(THeaInlVal.y, supHea.T_in) annotation (Line(points={{-77,52},{-66,52},
          {-66,62},{-42,62}},                                                                       color={0,0,127}));
  connect(couHea.heaPorLoa, building.heaPorHea) annotation (Line(points={{40,46},
          {40,22},{46,22},{46,-23},{74,-23}}, color={191,0,0}));
  connect(building.Q_flowHeaReq, couHea.Q_flowReq) annotation (Line(points={{95,
          -24},{100,-24},{100,44},{46,44},{46,48},{28,48}}, color={0,0,127}));
  connect(building.m_flowHeaLoa, couHea.m_flow2) annotation (Line(points={{95,-27},
          {106,-27},{106,72},{16,72},{16,64},{28,64}}, color={0,0,127}));
  connect(TSetCHWS.y, coo.TSet)
    annotation (Line(points={{-77,-42},{-48,-42},{-48,-50},{-12,-50}},
                                                   color={0,0,127}));
  connect(tra.y, souDis.T_in)
    annotation (Line(points={{-79,30},{-62,30}}, color={0,0,127}));
  connect(souDis.ports[1], TDisSup.port_a)
    annotation (Line(points={{-40,26},{-40,10},{-38,10}}, color={0,127,255}));
  connect(TDisSup.port_b, coo.port_a1) annotation (Line(points={{-18,10},{-18,-44},
          {-10,-44}}, color={0,127,255}));
  connect(coo.port_b1, TDisRet.port_a) annotation (Line(points={{10,-44},{14,-44},
          {14,-14},{18,-14}}, color={0,127,255}));
  connect(TDisRet.port_b, sinDis.ports[1])
    annotation (Line(points={{38,-14},{38,14},{16,14}}, color={0,127,255}));
  connect(coo.port_b2, pumBui.port_a) annotation (Line(points={{-10,-56},{-16,-56},
          {-16,-96},{-30,-96}}, color={0,127,255}));
  connect(exp.port_a, pumBui.port_a) annotation (Line(points={{-66,-80},{-32,-80},
          {-32,-96},{-30,-96}}, color={0,127,255}));
  connect(pumBui.port_b, hea.port_a) annotation (Line(points={{-30,-116},{-28,-116},
          {-28,-132},{34,-132},{34,-96},{52,-96},{52,-94}}, color={0,127,255}));
  connect(hea.port_b, coo.port_a2) annotation (Line(points={{72,-94},{84,-94},{84,
          -98},{108,-98},{108,-56},{10,-56}}, color={0,127,255}));
  connect(building.Q_flowCooAct[1], Q_flowCooAct1[1])
    annotation (Line(points={{95,-39},{154,-39},{154,-68}}, color={0,0,127}));
  connect(hea.u, Q_flowCooAct1[1]) annotation (Line(points={{50,-86.8},{42,-86.8},
          {42,-68},{154,-68}}, color={0,0,127}));
  connect(Q_flowCooAct1[1], pumBui.m_flow_in) annotation (Line(points={{154,-68},
          {154,-164},{-64,-164},{-64,-106},{-42,-106}}, color={0,0,127}));
  annotation (
  Documentation(info=\"<html>
  <p>
  This example illustrates the use of
  <a href=\\\"modelica://Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling\\\">
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling</a>
  to transfer heat from a fluid stream to a simplified building model consisting in two heating loads and one cooling
  load as described in
  <a href=\\\"modelica://Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.RCBuilding\\\">
  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.RCBuilding</a>.
  </p>
  </html>\"),
  Diagram(
  coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{140,80}})),
  __Dymola_Commands(file=\"Resources/Scripts/Dymola/Applications/DHC/Loads/Examples/CouplingRC.mos\"
        \"Simulate and plot\"));
        

end ETS_RC;
"));
  end ETS_RC;
end ETS_RC;
