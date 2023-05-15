within CRWT;
block WindSource "Wind speed ramp with random component"
  extends WindPower.Common.Icons.Blocks.WindSource;
  parameter Modelica.Units.SI.Velocity initWindSpeed=3
    "Wind speed during initialization"
    annotation (Dialog(group="Averaged Wind Component"));
  parameter Modelica.Units.SI.Velocity finalWindSpeed=13 "Final wind speed"
    annotation (Dialog(group="Averaged Wind Component"));
  parameter Modelica.Units.SI.Time riseTime=300
    "Rise time from initial wind speed to final wind speed"
    annotation (Dialog(group="Averaged Wind Component"));

  parameter Modelica.Units.SI.Period samplePeriod=1
    "Period for sampling the raw random numbers"
    annotation (Dialog(group="Random Wind Component"));
  parameter Real rand_min=1 "Lower limit for random part" annotation (Dialog(group="Random Wind Component"));
  parameter Real rand_max=5 "Upper limit for random part" annotation (Dialog(group="Random Wind Component"));

  Modelica.Blocks.Sources.Ramp averagedWindSpeed(
    duration=riseTime,
    height=finalWindSpeed - initWindSpeed,
    offset=initWindSpeed) annotation (Placement(transformation(extent={{-60,-10},
            {-40,10}})));
WindPower.Common.Interfaces.VelocityOutput windSpeed annotation (Placement(
        transformation(extent={{100,-10},{120,10}}), iconTransformation(extent=
            {{100,-10},{120,10}})));

equation
  connect(averagedWindSpeed.y, windSpeed)
    annotation (Line(points={{-39,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>This simple wind speed model contains a ramp and a random component which can be parametrized to generate wind profiles.</p>
</html>"));
end WindSource;
