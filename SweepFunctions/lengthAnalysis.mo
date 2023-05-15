within CRWT.SweepFunctions;
function lengthAnalysis
  input String pathToModel = "CRWT.ContraRotating";
  input String[3] parametersToSweep = {"contra_2MW_withPitch.r_t","contra_2MW_withPitch.J_t","comparisonMetric"};
  input Integer numSim = 20;
  input Real startVal = 20;
  input Real endVal = 60;
protected
  Real [:] lengthValues;
  Real [20] inertiaValues;
  Real [20] compVals;
algorithm
  lengthValues := linspace(
    startVal,
    endVal,
    numSim);
  for i in 1:numSim loop
    inertiaValues[i] := 8.6e6*((18+2*i)/40)^3;
    compVals[i] := lengthValues[i]/inertiaValues[i];
  end for;
  translateModel(pathToModel);
  for i in 1:numSim loop
    simulateExtendedModel(pathToModel, stopTime=500, method="dassl",
    resultFile="lengthSweep",
    initialNames={parametersToSweep[3]},
    initialValues={compVals[i]},
    finalNames={parametersToSweep[3]});
    //initialNames={parametersToSweep[1],parametersToSweep[2]},
    //initialValues={lengthValues[i],inertiaValues[i]},
    //finalNames={parametersToSweep[1],parametersToSweep[2],parametersToSweep[3]});
    plot({"P_t"},plotInAll=true);
  end for;
end lengthAnalysis;
