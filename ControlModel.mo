within CRWT;
model ControlModel
  "Generator-Side control of a wind turbine: Maximum Power Point Tracking via Optimal Torque and Pitch Control"
  extends Modelica.Icons.Example;

  WindPower.Turbines.Mechanical.Turbine_cp_Polynomial turbine(redeclare
      WindPower.Turbines.Mechanical.Records.Data.cp_Polynomial.Demo1_2MW_withPitch
      data, w(start=windSource.initWindSpeed*turbine.data.lambda_opt/turbine.data.r_t))
    annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
WindPower.ElectricMachines.PSM.ElectroMechanical.Linear linearPSM(redeclare
      WindPower.ElectricMachines.PSM.ElectroMechanical.Records.Data.Demo1_PSM_Linear_2MW
      data) annotation (Placement(transformation(extent={{0,30},{-20,50}})));
WindPower.PowerElectronics.Inverters.Averaged.ConstantEfficiency averagedInverter(
      redeclare
      WindPower.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant100percent
      data) annotation (Placement(transformation(extent={{40,30},{20,50}})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=2.4e-3, v(start=5.4e3))
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={80,40})));
WindPower.PowerElectronics.Inverters.PWM.NoModulation noModulation
    annotation (Placement(transformation(extent={{40,-60},{60,-40}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage u_DC(V=5.4e3)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={100,40})));

  WindPower.Turbines.Controllers.PitchControl pitchControl(redeclare
      WindPower.Turbines.Controllers.Records.Data.PitchControl.Demo1_2MW_withPitch
                                                                         data)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,-50})));
  CRWT.WindSource windSource(
    finalWindSpeed=15) annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  WindPower.Turbines.Controllers.MPPT_OT mppt(redeclare
      WindPower.Turbines.Mechanical.Records.Data.cp_Polynomial.Demo1_2MW_withPitch data)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-26,-50})));
WindPower.ElectricMachines.PSM.Controllers.TorqueToCurrent torqueController(redeclare
    WindPower.ElectricMachines.PSM.Controllers.Records.Base.Currents
                                                           data(
    simpleCurrentTuning=false,
    kp_d_set=3.75,
    Ti_d_set=0.24,
    kp_q_set=3.75,
    Ti_q_set=0.24,
    redeclare WindPower.ElectricMachines.PSM.ElectroMechanical.Records.Data.Demo1_PSM_Linear_2MW machineData))
  annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
  WindPower.Turbines.Interfaces.Adapters.FromBus.TurbineSpeed turbineSpeed
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={-50,0})));
equation

  connect(averagedInverter.plug, linearPSM.plug_p) annotation (Line(points={{20,40},{0,40}}, color={0,0,255}));
  connect(averagedInverter.pin_p, capacitor.p)
    annotation (Line(points={{40,46},{50,46},{50,60},{80,60},{80,50}}, color={0,0,255}));
  connect(averagedInverter.pin_n, capacitor.n)
    annotation (Line(points={{40,34},{50,34},{50,20},{80,20},{80,30}}, color={0,0,255}));
  connect(noModulation.normalizedPhaseVoltages, averagedInverter.normalizedPhaseVoltages)
    annotation (Line(points={{61,-50},{61,-50},{64,-50},{64,40},{42,40}}, color={0,0,127}));
  connect(linearPSM.electricDriveBus, averagedInverter.electricDriveBus)
    annotation (Line(
      points={{-10,30},{-10,30},{-10,0},{30,0},{30,16},{30,30}},
      color={0,86,166},
      thickness=0.5));
  connect(noModulation.electricDriveBus, linearPSM.electricDriveBus)
    annotation (Line(
      points={{50,-60},{50,-70},{30,-70},{30,0},{-10,0},{-10,30}},
      color={0,86,166},
      thickness=0.5));
  connect(turbine.flange, linearPSM.flange) annotation (Line(points={{-40,40},{-40,40},{-20,40}}, color={0,0,0}));
  connect(pitchControl.pitchAngle, turbine.pitchAngle)
    annotation (Line(points={{-91,-50},{-91,-50},{-100,-50},{-100,-32},{-70,-32},{-70,32},{-62,32}},   color={0,0,127}));
  connect(windSource.windSpeed, turbine.windSpeed) annotation (Line(points={{-79,40},{-70,40},{-62,40}}, color={0,0,127}));
  connect(torqueController.actuatingVoltages, noModulation.phaseVoltages)
    annotation (Line(points={{21,-50},{38,-50}}, color={0,0,127}));
  connect(torqueController.electricDriveBus, linearPSM.electricDriveBus)
    annotation (Line(
      points={{10,-60},{10,-70},{30,-70},{30,0},{-10,0},{-10,30}},
      color={0,86,166},
      thickness=0.5));
  connect(mppt.torqueOutput, torqueController.desiredTorque)
    annotation (Line(points={{-15,-50},{-2,-50}},         color={0,0,127}));
  connect(u_DC.p, capacitor.p) annotation (Line(points={{100,50},{100,60},{80,60},{80,50}}, color={0,0,255}));
  connect(u_DC.n, capacitor.n) annotation (Line(points={{100,30},{100,30},{100,20},{80,20},{80,30}}, color={0,0,255}));
  connect(ground.p, u_DC.n) annotation (Line(points={{100,10},{100,10},{100,20},{100,30}}, color={0,0,255}));
  connect(pitchControl.turbineSpeed, mppt.angularVelocityInput)
    annotation (Line(points={{-68,-50},{-38,-50}},           color={0,0,127}));
  connect(turbineSpeed.turbineBus, turbine.turbineBus)
    annotation (Line(
      points={{-50,6},{-50,30}},
      color={204,204,204},
      thickness=0.5));
  connect(turbineSpeed.y, mppt.angularVelocityInput) annotation (Line(points={{-50,-6.6},{-50,-50},{-38,-50}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-120,-100},{120,100}},
        initialScale=0.1), graphics={
        Rectangle(
          extent={{-116,-12},{88,-90}},
          radius=5,
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-108,-20},{-60,-80}},
          radius=5,
          lineThickness=0.5,
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          fillColor={245,245,245},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-6,-18},{70,-80}},
          radius=5,
          lineThickness=0.5,
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          fillColor={245,245,245},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{24,-22},{0,-28}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={235,240,240},
          fillPattern=FillPattern.Solid,
          textString="Torque Control"),
        Text(
          extent={{-96,-22},{-74,-28}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={235,240,240},
          fillPattern=FillPattern.Solid,
          textString="Pitch Control"),
        Text(
          extent={{-52,-80},{-12,-92}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={235,240,240},
          fillPattern=FillPattern.Solid,
          textString="Generator-Side Control")}),
    experiment(StopTime=500, __Dymola_NumberOfIntervals=50000),
    __Dymola_Commands(file="modelica://WindPower/Resources/Scripts/plot/GeneratorSideControl.mos" "plot"),
    Documentation(info="<html>
<p>This example demonstrates maximum power point tracking and pitch control of a wind turbine.</p>
<p>Refer to <a href=\"WindPower.UsersGuide.Fundamentals\">Fundamentals</a> for a description of this specific area of operation of a wind turbine.</p>

<p><hr><h4>Tutorials</h4></p>
<p>Find more information about this example in the following tutorial: <a href=\"WindPower.UsersGuide.Tutorials.PitchControl\">PitchControl</a></p>
</html>"));
end ControlModel;
