within CRWT.ContraTurbineComponents;
record Contra_2MW_withPitch "Contra_2MW_withPitch"
  import WindPower.Functions.divNoZero;
  extends WindPower.Common.Icons.DataHandling.Record;
  extends WindPower.Turbines.Mechanical.Records.Base.Turbine_cp_Polynomial(
    c1=0.73,
    c2=151,
    c3=0.58,
    c4=0.002,
    c5=13.2,
    c6=18.4,
    x=2.14,
    f=divNoZero(1, (lambda - 0.02*beta_deg)) - 0.003/(beta_deg^3 + 1),
    rho=1.293,
    r_t=40,
    J_t=8.6e6,
    P_t_nom=2e6,
    lambda_opt=1/((c2/c6 + c5)/c2 + 0.003),
    cp_max=0.441);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Exemplary data for a 2MW turbine with pitch system. Values are taken from <a href=\"WindPower.UsersGuide.Literature\">[Sch15, Chapter 24.3.1.2, Table 24.1 and Table 24.3]</a></p>
</html>"));
end Contra_2MW_withPitch;
