within CRWT.TurbineComponents;
partial model PartialTurbine
  extends WindPower.Common.Icons.Models.Turbine;

  // Parameters
  parameter Modelica.Units.SI.Radius r_t "Turbine radius"
    annotation (Dialog(group="Turbine"));
  parameter Modelica.Units.SI.Density rho "Density of medium (usually air)"
    annotation (Dialog(group="Turbine"));
  parameter Modelica.Units.SI.Inertia J_t "Inertia of the turbine"
    annotation (Dialog(group="Turbine"));
  Modelica.Units.SI.AngularVelocity w "Angular velocity of the turbine"
    annotation (Dialog(showStartAttribute=true));

  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  CRWT.TurbineComponents.TurbineTorque turbineTorque(rho=rho, r_t=r_t)
    annotation (Placement(transformation(extent={{10,-10},{30,10}})));
WindPower.Common.Interfaces.AngleInput pitchAngle annotation (Placement(
        transformation(extent={{-140,-100},{-100,-60}}), iconTransformation(
          extent={{-140,-100},{-100,-60}})));
WindPower.Common.Interfaces.VelocityInput windSpeed annotation (Placement(
        transformation(extent={{-140,-20},{-100,20}}), iconTransformation(
          extent={{-140,-20},{-100,20}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J_t, w=w)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  replaceable
    WindPower.Turbines.Mechanical.CoreElements.Components.Partial.PowerCoefficient
    powerCoefficient constrainedby
    WindPower.Turbines.Mechanical.CoreElements.Components.Partial.PowerCoefficient
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})),
      choicesAllMatching=true);
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={80,10})));

  WindPower.Common.Blocks.TipSpeedRatio tipSpeedRatio(r_t=r_t)
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
WindPower.Common.Interfaces.NormalizedOutput cp annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-110})));
WindPower.Common.Interfaces.NormalizedOutput lambda
    "Connector of Real output signal" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,-110})));
equation
  connect(turbineTorque.windSpeed, windSpeed) annotation (Line(points={{8,0},{8,0},{-120,0}},               color={0,0,127}));
  connect(inertia.flange_b, flange) annotation (Line(points={{60,0},{60,0},{100,0}}, color={0,0,0}));
  connect(inertia.flange_b, speedSensor.flange) annotation (Line(points={{60,0},{60,0},{80,0}},  color={0,0,0}));
  connect(powerCoefficient.cp, turbineTorque.powerCoefficient)
    annotation (Line(points={{-19,-30},{-12,-30},{-12,-6},{8,-6}},
                                                                 color={0,0,127}));
  connect(pitchAngle, powerCoefficient.beta)
    annotation (Line(points={{-120,-80},{-50,-80},{-50,-36},{-42,-36}},                 color={0,0,127}));
  connect(tipSpeedRatio.windSpeed, windSpeed)
    annotation (Line(points={{-82,24},{-90,24},{-90,0},{-120,0}},                  color={0,0,127}));
  connect(tipSpeedRatio.turbineSpeed, speedSensor.w)
    annotation (Line(points={{-82,36},{-90,36},{-90,60},{80,60},{80,21}},
                                                                       color={0,0,127}));
  connect(tipSpeedRatio.lambda, powerCoefficient.lambda)
    annotation (Line(points={{-59,30},{-59,30},{-56,30},{-50,30},{-50,-24},{-42,-24}},
                                                                           color={0,0,127}));
  connect(turbineTorque.flange_a, inertia.flange_a) annotation (Line(points={{30,0},{36,0},{40,0}},
                                                                                             color={0,0,0}));
  connect(powerCoefficient.cp, cp) annotation (Line(points={{-19,-30},{-19,-30},{-12,-30},{-12,-90},{-60,-90},{-60,-110}},
                                                                                                             color={0,0,127}));
  connect(tipSpeedRatio.lambda, lambda)
    annotation (Line(points={{-59,30},{-50,30},{-50,-14},{-8,-14},{-8,-90},{60,-90},{60,-110}},
                                                                                             color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={Polygon(
          points={{-56,14},{-56,14}},
          lineColor={28,108,200},
          fillPattern=FillPattern.Sphere,
          fillColor={28,108,200})}),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>This model is based on <a href=\"WindPower.Turbines.Mechanical.CoreElements.Components.TurbineTorque\">TurbineTorque</a>.</p>
<p>Aditionnally </p>
<ul>
<li>the inertia of the rotor is introduced</li>
<li>the tip-speed ratio is calculated</li>
<li>the power coefficient is calculated from the chosen model (the model of choice must be redeclared)</li>
</ul>
</html>"));
end PartialTurbine;
