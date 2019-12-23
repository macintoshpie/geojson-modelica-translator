within Buildings.Applications.DHC.Loads.Examples;
model CouplingETS_RC_Yanfei_Cooling
  "Example illustrating the coupling of a RC building model to a Energy Transfer Station (ETS) cooling model"
  import Buildings;
  import Buildings.Applications.DHC.EnergyTransferStations.CoolingIndirect;
  extends Modelica.Icons.Example;

  package Medium = Buildings.Media.Water;

  parameter Modelica.SIunits.MassFlowRate mDis_flow_nominal = 0.5
    "Nominal mass flow rate on district-side (primary)";
  parameter Modelica.SIunits.MassFlowRate mBui_flow_nominal = 0.5
    "Nominal mass flow rate on building-side (secondary)";

  parameter Modelica.SIunits.MassFlowRate m1_flow_nominal=0.5 "kg/s";

  parameter Real cp = 4.2 "J/kg-k";

  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{88,50},{68,70}})));

  Buildings.Applications.DHC.Loads.Examples.BaseClasses.RCBuilding building(
      Q_flowCoo_nominal={1000}, Q_flowHea_nominal={2000,1000})
    annotation (Placement(transformation(extent={{-18,18},{8,44}})));

  Buildings.Applications.DHC.EnergyTransferStations.CoolingIndirect coo(
    redeclare package Medium = Buildings.Media.Water,
    m1_flow_nominal=0.5,
    m2_flow_nominal=0.5,
    dpValve_nominal=10,
    dp1_nominal=1000,
    dp2_nominal=500,
    use_Q_flow_nominal=true,
    Q_flow_nominal=1000,
    T_a1_nominal=7 + 273.15,
    T_a2_nominal=30 + 273.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-84,-46})));
  Modelica.Blocks.Sources.Constant TSetCHWS(k=273.15 + 7)
    "Setpoint temperature for building chilled water supply"
    annotation (Placement(transformation(extent={{-182,-96},{-162,-76}})));
  Modelica.Blocks.Sources.Trapezoid tra(
    amplitude=1.5,
    rising(displayUnit="h") = 10800,
    width(displayUnit="h") = 10800,
    falling(displayUnit="h") = 10800,
    period(displayUnit="h") = 43200,
    offset=273 + 3.5)
    "District supply temperature trapezoid signal"
    annotation (Placement(transformation(extent={{-216,-44},{-196,-24}})));
  Buildings.Fluid.Sources.Boundary_pT souDis(
    p(displayUnit="Pa") = 300000 + 800,
    use_T_in=true,
    nPorts=1,
    redeclare package Medium = Buildings.Media.Water,
    T=278.15)
    "District (primary) source"
    annotation (Placement(transformation(extent={{-184,-48},{-164,-28}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort TDisSup(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    T_start=278.15)
    "District-side (primary) supply temperature sensor"
    annotation (Placement(transformation(extent={{-140,-56},{-120,-36}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort TDisRet(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    T_start=287.15)
    "District-side (primary) return temperature sensor"
    annotation (Placement(transformation(extent={{-118,-8},{-138,12}})));
  Buildings.Fluid.Sources.Boundary_pT sinDis(
    nPorts=1,
    redeclare package Medium = Buildings.Media.Water,
    p=300000,
    T=287.15)
    "District-side (primary) sink"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-170,14})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pumpBuiding(
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    nominalValuesDefineDefaultPressureCurve=true,
    dp_nominal=6000,
    m_flow_nominal=0.5,
    redeclare package Medium = Buildings.Media.Water)
    "Building-side (secondary) pump" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-42,-82})));
  Buildings.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium,
      V_start=1000)
    "Expansion tank"
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-68,-114})));
  Buildings.Fluid.HeatExchangers.HeaterCooler_u2 CoolingUnit(
    m_flow_nominal=0.5,
    dp_nominal=2,
    redeclare package Medium = Buildings.Media.Water,
    Q_flow_nominal=-1000)  "A Simplified Air Terminal of Cooling Unit"
    annotation (Placement(transformation(extent={{-10,-46},{20,-18}})));
  Modelica.Blocks.Math.Gain gain(k=1)
    annotation (Placement(transformation(extent={{62,4},{82,24}})));
  Modelica.Blocks.Math.Gain gain1(k=-1/(cp*(16 - 7)))
    annotation (Placement(transformation(extent={{42,-118},{22,-98}})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
    annotation (Placement(transformation(extent={{-64,36},{-54,46}})));
  Modelica.Blocks.Sources.CombiTimeTable QCoo(
    timeScale=3600,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    table=[0,-120; 6,-100; 9,-200; 12,-210; 18,-150; 24,-100; 30,-120; 36,-110;
        42,-200; 48,-120; 54,-150])
    "Cooling demand"
    annotation (Placement(transformation(extent={{-128,38},{-108,58}})));
equation
  connect(weaDat.weaBus, building.weaBus) annotation (Line(
      points={{68,60},{40,60},{40,52},{18,52},{18,44},{-4.87,44}},
      color={255,204,51},
      thickness=0.5));
  connect(TSetCHWS.y, coo.TSet)
    annotation (Line(points={{-161,-86},{-84,-86},{-84,-58}},
                                                   color={0,0,127}));
  connect(tra.y, souDis.T_in)
    annotation (Line(points={{-195,-34},{-186,-34}},
                                                 color={0,0,127}));
  connect(souDis.ports[1], TDisSup.port_a)
    annotation (Line(points={{-164,-38},{-164,-46},{-140,-46}},
                                                          color={0,127,255}));
  connect(TDisSup.port_b, coo.port_a1) annotation (Line(points={{-120,-46},{-120,
          -56},{-90,-56}},
                      color={0,127,255}));
  connect(coo.port_b1, TDisRet.port_a) annotation (Line(points={{-90,-36},{-90,2},
          {-118,2}},          color={0,127,255}));
  connect(TDisRet.port_b, sinDis.ports[1])
    annotation (Line(points={{-138,2},{-138,14},{-160,14}},
                                                        color={0,127,255}));
  connect(coo.port_b2, pumpBuiding.port_a) annotation (Line(points={{-78,-56},{
          -68,-56},{-68,-82},{-52,-82}},
                                       color={0,127,255}));
  connect(exp.port_a, pumpBuiding.port_a)
    annotation (Line(points={{-68,-104},{-68,-82},{-52,-82}},
                                                      color={0,127,255}));
  connect(pumpBuiding.port_b, CoolingUnit.port_a) annotation (Line(points={{-32,-82},
          {-18,-82},{-18,-32},{-10,-32}},   color={0,127,255}));
  connect(CoolingUnit.port_b, coo.port_a2) annotation (Line(points={{20,-32},{
          34,-32},{34,2},{-60,2},{-60,-36},{-78,-36}}, color={0,127,255}));
  connect(building.Q_flowCooAct[1], gain.u)
    annotation (Line(points={{9.3,19.3},{32,19.3},{32,14},{60,14}},
                                                            color={0,0,127}));
  connect(gain.y, CoolingUnit.u) annotation (Line(points={{83,14},{96,14},{96,
          -12},{-38,-12},{-38,-21.92},{-13,-21.92}},
                                                color={0,0,127}));
  connect(gain1.y, pumpBuiding.m_flow_in) annotation (Line(points={{21,-108},{
          -42,-108},{-42,-94}},
                             color={0,0,127}));
  connect(gain1.u, gain.u) annotation (Line(points={{44,-108},{44,14},{60,14}},
        color={0,0,127}));
  connect(prescribedHeatFlow.port, building.heaPorCoo[1]) annotation (Line(
        points={{-54,41},{-42,41},{-42,21.9},{-18,21.9}}, color={191,0,0}));
  connect(QCoo.y[1], prescribedHeatFlow.Q_flow) annotation (Line(points={{-107,48},
          {-92,48},{-92,44},{-76,44},{-76,41},{-65.1,41}},
                                     color={0,0,127}));
  annotation (
  Documentation(info="<html>
  <p>
  This example illustrates the use of a house model (RC) from (Buildings.Applications.DHC.Loads.BaseClasses.RCBuilding), 
  and a Energy Transfer Station (ETS) model from (Buildings.Applications.DHC.EnergyTransferStations.CoolingIndirect). 
  The house transfers heat from a a simplified air terminal to the ETS, under cooling mode.
  </p>
  </html>"),
  Diagram(
  coordinateSystem(preserveAspectRatio=false, extent={{-220,-140},{100,80}})),
  __Dymola_Commands(file="Resources/Scripts/Dymola/Applications/DHC/Loads/Examples/CouplingRC.mos"
        "Simulate and plot"),
    Icon(coordinateSystem(extent={{-220,-140},{100,80}})));
end CouplingETS_RC_Yanfei_Cooling;
