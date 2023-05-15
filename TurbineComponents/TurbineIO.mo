within CRWT.TurbineComponents;
model TurbineIO "Energy based, polynomial approximation for cp"

  // Interface
  extends WindPower.Turbines.Mechanical.Interfaces.Base;

  // Icon
  extends WindPower.Common.Icons.Models.Turbine;
  extends WindPower.Common.Icons.SubIcons.Polynomial_LeftCorner;

  CRWT.TurbineComponents.TurbineInnerIO coreElement(
    c1=data.c1,
    c2=data.c2,
    c3=data.c3,
    c4=data.c4,
    c5=data.c5,
    c6=data.c6,
    x=data.x,
    r_t=data.r_t,
    rho=data.rho,
    J_t=data.J_t,
    f=data.f,
    w=w) annotation (Placement(transformation(extent={{-20,-20},{20,20}})),
      __Dymola_choicesAllMatching=true);

  // Auxiliary variables
  Modelica.Units.SI.Angle beta=angleSource.y;
  Modelica.Units.SI.DimensionlessRatio lambda=tipSpeedRatio.lambda;
  Modelica.Units.SI.AngularVelocity w "Angular velocity of the turbine"
    annotation (Dialog(showStartAttribute=true));

  replaceable WindPower.Turbines.Mechanical.Records.Data.cp_Polynomial.Default data constrainedby
    WindPower.Turbines.Mechanical.Records.Base.Parameters.cp_Polynomial(lambda=lambda, beta=beta)
    annotation (choicesAllMatching, Placement(transformation(extent={{70,70},{90,90}})));

  WindPower.Common.Blocks.TipSpeedRatio tipSpeedRatio(r_t=data.r_t)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-20,80})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,30})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor_turbine
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,-20})));
  WindPower.Turbines.Interfaces.Adapters.ToBus.TurbineSpeed turbineSpeed_toBus
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={60,-60})));
  WindPower.Turbines.Interfaces.Adapters.ToBus.TipSpeedRatio tipSpeedRatio_toBus
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={20,-60})));
  WindPower.Turbines.Interfaces.Adapters.ToBus.PowerCoefficient powerCoefficient_toBus
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={-20,-60})));
  WindPower.Turbines.Interfaces.Adapters.ToBus.WindSpeed windSpeed_toBus
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={-40,-60})));

  Modelica.Blocks.Interfaces.RealOutput P_w
    annotation (Placement(transformation(extent={{80,20},{100,40}})));
  Modelica.Blocks.Interfaces.RealOutput P_t
    annotation (Placement(transformation(extent={{80,-40},{100,-20}})));
equation
  connect(coreElement.flange, flange) annotation (Line(points={{20,0},{20,0},{100,0}}, color={0,0,0}));
  connect(tipSpeedRatio.turbineSpeed, speedSensor.w)
    annotation (Line(points={{-32,74},{-32,74},{-36,74},{-36,78},{-38,78},{-38,60},{50,60},{50,41}}, color={0,0,127}));
  connect(speedSensor.flange, flange) annotation (Line(points={{50,20},{50,0},{100,0}}, color={0,0,0}));
  connect(windSpeed, tipSpeedRatio.windSpeed)
    annotation (Line(points={{-120,0},{-40,0},{-40,82},{-36,82},{-36,86},{-32,86}}, color={0,0,127}));
  connect(coreElement.flange, flange) annotation (Line(points={{20,0},{62,0},{100,0}}, color={0,0,0}));
  connect(coreElement.windSpeed, windSpeed) annotation (Line(points={{-24,0},{-80,0},{-120,0}},         color={0,0,127}));
  connect(speedSensor_turbine.flange, flange) annotation (Line(points={{60,-10},{60,0},{100,0}}, color={0,0,0}));
  connect(turbineSpeed_toBus.u, speedSensor_turbine.w) annotation (Line(points={{60,-52.8},{60,-31}}, color={0,0,127}));
  connect(turbineSpeed_toBus.turbineBus, turbineBus)
    annotation (Line(
      points={{60,-66},{60,-66},{60,-100},{0,-100}},
      color={204,204,204},
      thickness=0.5));
  connect(tipSpeedRatio_toBus.turbineBus, turbineBus)
    annotation (Line(
      points={{20,-66},{20,-100},{0,-100}},
      color={204,204,204},
      thickness=0.5));
  connect(powerCoefficient_toBus.turbineBus, turbineBus)
    annotation (Line(
      points={{-20,-66},{-20,-66},{-20,-100},{0,-100}},
      color={204,204,204},
      thickness=0.5));
  connect(coreElement.cp, powerCoefficient_toBus.u)
    annotation (Line(points={{-12,-22},{-12,-22},{-12,-40},{-20,-40},{-20,-52.8}}, color={0,0,127}));
  connect(coreElement.lambda, tipSpeedRatio_toBus.u)
    annotation (Line(points={{12,-22},{12,-22},{12,-40},{20,-40},{20,-52.8}}, color={0,0,127}));
  connect(windSpeed_toBus.turbineBus, turbineBus)
    annotation (Line(
      points={{-40,-66},{-40,-66},{-40,-100},{0,-100}},
      color={204,204,204},
      thickness=0.5));
  connect(windSpeed, windSpeed_toBus.u) annotation (Line(points={{-120,0},{-80,0},{-40,0},{-40,-52.8}},         color={0,0,127}));

  connect(angleSource.y, coreElement.pitchAngle)
    annotation (Line(points={{-59,-80},{-60,-80},{-60,-80},{-54,-80},{-50,-80},{-50,-16},{-24,-16}}, color={0,0,127}));
  connect(coreElement.P_w, P_w) annotation (Line(points={{18,-6},{64,-6},{64,30},
          {90,30}}, color={0,0,127}));
  connect(coreElement.P_t, P_t) annotation (Line(points={{18,-14},{44,-14},{44,-34},
          {74,-34},{74,-30},{90,-30}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{12,90},{48,68}},
          lineColor={95,95,95},
          pattern=LinePattern.Dot),Text(
          extent={{14,88},{38,80}},
          lineColor={95,95,95},
          pattern=LinePattern.Dot,
          fillPattern=FillPattern.Solid,
          textString="text layer"),Text(
          extent={{14,78},{44,70}},
          lineColor={95,95,95},
          pattern=LinePattern.Dot,
          horizontalAlignment=TextAlignment.Left,
          textString="beta=angleSource.y
lambda=tipSpeedRatio.lambda"),Line(
          points={{0,-8},{0,8}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={-2,80},
          rotation=90),Line(
          points={{-2,0},{2,0}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={6,80},
          rotation=90),Line(
          points={{2,3},{1.07002e-016,-3}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={9,82},
          rotation=270),Line(
          points={{-2,3},{0,-3}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={9,78},
          rotation=270),Line(
          points={{0,-8},{1.22465e-016,6}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={54,80},
          rotation=90),Line(
          points={{-2,0},{2,0}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={62,80},
          rotation=90),Line(
          points={{2,3},{1.07002e-016,-3}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={65,82},
          rotation=270),Line(
          points={{-2,3},{0,-3}},
          color={95,95,95},
          pattern=LinePattern.Dot,
          smooth=Smooth.None,
          origin={65,78},
          rotation=270)}),
    Documentation(info="<html>
<p>This is an energy based model of a wind turbine that uses a polynomial to approximate the power coefficient <img src=\"modelica://WindPower/Resources/Images/equations/cp.png\"/>.</p>
<p>See the <a href=\"WindPower.Turbines.Mechanical.CoreElements.Turbine_cp_Polynomial\">core elements documentation</a> for details.</p>
</html>"));
end TurbineIO;
