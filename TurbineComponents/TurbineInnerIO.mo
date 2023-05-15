within CRWT.TurbineComponents;
model TurbineInnerIO
  import WindPower.Turbines.Mechanical.Records.Base.Parameters;

  // Interface
  extends CRWT.TurbineComponents.PartialTurbine(
      redeclare
      WindPower.Turbines.Mechanical.CoreElements.Components.Polynomial_cp
      powerCoefficient(
      c1=c1,
      c2=c2,
      c3=c3,
      c4=c4,
      c5=c5,
      c6=c6,
      x=x,
      f=f));

  // Icon
  extends WindPower.Common.Icons.SubIcons.Polynomial_LeftCorner;

  // Parameters
  extends Parameters.cp_Polynomial(f=1);

  Modelica.Blocks.Interfaces.RealOutput P_w
    annotation (Placement(transformation(extent={{80,-40},{100,-20}})));
  Modelica.Blocks.Interfaces.RealOutput P_t
    annotation (Placement(transformation(extent={{80,-80},{100,-60}})));
equation

  connect(turbineTorque.Pw, P_w) annotation (Line(points={{29,-3},{36,-3},{36,-30},
          {90,-30}}, color={0,0,127}));
  connect(turbineTorque.Pt, P_t) annotation (Line(points={{29,-7},{34,-7},{34,-70},
          {90,-70}}, color={0,0,127}));
  annotation (Documentation(info="<html>
<p>This is an energy based model of a wind turbine that uses a polynomial to approximate the power coefficient <img src=\"modelica://WindPower/Resources/Images/equations/cp.png\"/>.</p>
<p>See the documentation of the individual component for details.</p>

<p><hr><h4>Limitations</h4></p>
If the tip-speed ratio is zero (if the turbine is standing still) the power coefficient will be zero, so the turbine is not able to accelerate.
Hence an initial speed must be provided.

</html>"));
end TurbineInnerIO;
