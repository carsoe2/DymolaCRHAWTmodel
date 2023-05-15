within CRWT.TurbineComponents;
model TurbineTorque
  "Calculates the torque at the turbine shaft from wind speed and pitch angle via an approximated power coefficient"

  // Icon
  extends WindPower.Common.Icons.Models.TurbineTorque;

  // Parameters
  parameter Modelica.Units.SI.Radius r_t "Turbine radius";
  parameter Modelica.Units.SI.Density rho "Density of medium (usually air)";

  Modelica.Units.SI.Power P_w "Wind power";
  Modelica.Units.SI.Power P_t
    "Actual Turbine power (maximum extractable wind power)";
  Modelica.Units.SI.Torque tau_t "Turbine torque";
  Modelica.Units.SI.AngularVelocity w_t "Angular velocity of the turbine";

  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (Placement(transformation(extent={{90,-10},{110,10}})));

WindPower.Common.Interfaces.NormalizedInput powerCoefficient annotation (
      Placement(transformation(extent={{-140,-80},{-100,-40}}),
        iconTransformation(extent={{-140,-80},{-100,-40}})));
WindPower.Common.Interfaces.VelocityInput windSpeed annotation (Placement(
        transformation(extent={{-140,-20},{-100,20}}), iconTransformation(
          extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput Pw
    annotation (Placement(transformation(extent={{80,-40},{100,-20}})));
  Modelica.Blocks.Interfaces.RealOutput Pt
    annotation (Placement(transformation(extent={{80,-80},{100,-60}})));
equation
  tau_t = -flange_a.tau;
  w_t = der(flange_a.phi);

  P_w = 1/2 * rho * r_t^2 * Modelica.Constants.pi * windSpeed^3;
  P_t =P_w*powerCoefficient;
  tau_t = WindPower.Functions.divNoZero(P_t, w_t);
  Pw=P_w;
  Pt=P_t;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{-56,14},{-56,14}},
          lineColor={28,108,200},
          fillPattern=FillPattern.Sphere,
          fillColor={28,108,200})}),
                           Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Basic model of a wind turbine that uses the power coefficient <img src=\"modelica://WindPower/Resources/Images/equations/equation-HbrqE19I.png\" alt=\"c_p = f(lambda, beta)\"/> to calculate the torque which is generated due to the actual wind, as described in <a href=\"WindPower.UsersGuide.Fundamentals\">Fundamentals</a>. </p>
<p><img src=\"modelica://WindPower/Resources/Images/equations/cp.png\" alt=\"c_p\"/> is provided as input to this model and it can be calculated in <a href=\"WindPower.Turbines.Mechanical.CoreElements.Components.Polynomial_cp\">WindPower.Turbines.Mechanical.CoreElements.Components.Polynomial_cp</a> or read from a table with <a href=\"WindPower.Turbines.Mechanical.CoreElements.Components.Table_cp\">WindPower.Turbines.Mechanical.CoreElements.Components.Table_cp</a>. </p>
</html>"));
end TurbineTorque;
