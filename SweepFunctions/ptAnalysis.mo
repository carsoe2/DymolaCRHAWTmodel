within CRWT.SweepFunctions;
function ptAnalysis
  input String pathToModel = "CRWT.ContraRotating";
  input String[3] parametersToSweep = {"contraTurbine.data.r_t","contraTurbine.data.J_t","comparisonMetric"};
  input Integer numSim = 20;
  input Real startVal = 22;
  input Real endVal = 60;
  input Real simTime = 500;
protected
  Real [:] lengthValues;
  Real [20] inertiaValues;
algorithm
  lengthValues := linspace(
    startVal,
    endVal,
    numSim);
  for i in 1:numSim loop
    inertiaValues[i] := 8.6e6*((20+2*i)/40)^3;
    translateModel(pathToModel);
    simulateExtendedModel(pathToModel, stopTime=simTime, method="dassl",
    resultFile="lengthSweep",
    initialNames={parametersToSweep[1],parametersToSweep[2]},
    initialValues={lengthValues[i],inertiaValues[i]},
    finalNames={parametersToSweep[1],parametersToSweep[2],parametersToSweep[3]});
    plot({"P_t"},plotInAll=true);
  end for;
end ptAnalysis;
