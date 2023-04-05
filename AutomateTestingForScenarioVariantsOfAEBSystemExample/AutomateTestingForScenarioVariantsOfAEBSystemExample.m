%% Automate Testing for Scenario Variants of AEB System
% This example shows how to assess the functionality of an autonomous
% emergency braking (AEB) application by varying the European New Car
% Assessment Programme (Euro NCAP) Car-to-Pedestrian Nearside Child (CPNC)
% driving scenario. This example builds on the
% <docid:driving_ug#mw_b60cf0e1-2485-4e0f-8f1f-8c3300484139 Autonomous
% Emergency Braking with Sensor Fusion> example.

% Copyright 2022-2023 The MathWorks, Inc.

%% Introduction
% Autonomous emergency braking is an advanced active safety system that
% helps drivers avoid or mitigate collisions with other vehicles. The test
% space for autonomous driving functions is incredibly large, and it is not
% feasible to validate large numbers of scenarios with real-world driving
% tests. This example enables you to perform scenario-based testing of an
% AEB system by generating multiple scenario variants from a seed scenario.
%
% The example shows how to vary ego speed and collision point parameters to
% generate multiple variants of the Euro NCAP CPNC test scenario. The
% example also shows how to use iterative testing methodology in
% Simulink(R) *Test Manager* to test the generated variants. In this
% example, you:
%
% # *Explore the test bench model* &mdash; The model contains modules for
% the sensors and environment, sensor fusion and tracking, decision logic,
% controls, and vehicle dynamics.
% # *Review seed scenario and generate variants* &mdash; Review the CPNC
% seed scenario and generate variants of the seed scenario by varying the
% ego speed and the collision point between the ego vehicle and pedestrian.
% # *Perform iterative testing with scenario variants* &mdash; Configure
% *Test Manager* to simulate the AEB test bench model for each scenario
% variant using scripted iterative testing with Simulink Test&trade;,
% assess success criteria, and report results.
% # *Visualize the results and review the generated report* &mdash; Plot
% a grid-based visualization to review and analyze the results from
% iterative testing. You can also export the simulation results to a PDF
% and review the generated report.
%
%% Explore Test Bench Model
% This example reuses the |AEBTestBench| model from the
% <docid:driving_ug#mw_b60cf0e1-2485-4e0f-8f1f-8c3300484139 Autonomous
% Emergency Braking with Sensor Fusion> example.
%
% To explore the test bench model, open a working copy of the project
% example files. MATLAB(R) copies the files to an example folder so that
% you can edit them.
%%
helperDrivingProjectSetup("AutonomousEmergencyBraking.zip",workDir=pwd)
%% 
% To reduce Command Window output, turn off model predictive controller
% (MPC) update messages.
%%
mpcverbosity("off");
%%
% Open the system-level simulation test bench model. 
open_system("AEBTestBench")
%%
% This test bench model has these modules:
%
% * |Sensors and Environment| &mdash; Subsystem that specifies the road,
% actors, camera, and radar sensor used for simulation.
% * |Sensor Fusion and Tracking| &mdash; Algorithm model that fuses vehicle
% detections from the camera to those from the radar sensor.
% * |AEB Decision Logic| &mdash; Algorithm model that specifies the lateral
% and longitudinal decision logic that provides most important object (MIO)
% related information and ego vehicle reference path information to the
% controller.
% * |AEB Controller| &mdash; Algorithm model that specifies the steering
% angle and acceleration controls.
% * |Vehicle Dynamics| &mdash; Subsystem that specifies the dynamic model
% of the ego vehicle.
% * |Metrics Assessment| &mdash; Subsystem that assesses system-level
% behavior.
%
%%
% For more details on these components and simulating the test bench model,
% see the <docid:driving_ug#mw_b60cf0e1-2485-4e0f-8f1f-8c3300484139
% Autonomous Emergency Braking with Sensor Fusion> example.
%
% In this example, your focus is on automating the simulation runs for
% multiple variants of a seed scenario. You create the seed scenario using
% the *Driving Scenario Designer* app.
%
%% Review Seed Scenario and Generate Variants
% The |helperCreateSeedScenario| function generates a cuboid scenario that
% is compatible with the |AEBTestBench| model. This is an open-loop
% scenario containing two stationary target vehicles on a straight road and
% a child pedestrian. This is a Euro NCAP CPNC scenario, in which the ego
% vehicle collides with a pedestrian child who is running behind the
% obstruction from the nearside. The frontal structure of the vehicle
% hits the pedestrian.
% 
% Plot the open-loop scenario to see the interactions of the ego vehicle
% and the pedestrian child target.
%%
hFigSeedScenario = helperPlotScenario("helperCreateSeedScenario");
%%
% The ego vehicle, which is not under closed-loop control, collides with
% the pedestrian child. The goal of the closed-loop system is to avoid or
% mitigate the collision with the pedestrian actor. In the |AEBTestBench|
% model, the ego vehicle has the same initial speed and initial position as
% in the open-loop scenario.
%%
% Close the figure.
close(hFigSeedScenario)
%% Generate Variants of Seed Scenario
% The |helperCreateSeedScenario| function creates a seed scenario in which
% the speed of the ego vehicle is 60 km/h, and the collision point is at
% the front-left corner of the ego vehicle. To test this AEB CPNC scenario
% per Euro NCAP test protocols, you create multiple variants of the seed
% scenario by varying the ego speed and collision point. Use the
% |helperCreateSeedScenario| function to create the seed scenario.
%%
[seedScenario,egoVehicle] = helperCreateSeedScenario;

%% 
% Use the <docid:driving_ref#mw_eccfb10b-cc6e-41f1-bd15-cffb5eb0e6d9
% getScenarioDescriptor> function to create a |scenarioDescriptor| object
% that has the properties of the seed scenario. You can modify the
% properties of this object to generate scenario variations.

% Get the scenario descriptor from the seed scenario
scenarioDescriptor = getScenarioDescriptor(seedScenario,Simulator="DrivingScenario");
%% 
% Define the IDs of the ego vehicle and the target actor that collides with
% the ego vehicle.
egoID = egoVehicle.ActorID;
targetID = 2;
%%
% To generate scenario variations, the ego vehicle in the seed scenario
% must satisfy these criteria:
%
% * The ego vehicle must collide with the target vehicle.
% * The ego vehicle must have consistent motion properties.
% * The ego trajectory must have at least three waypoints.
% * The ego vehicle must travel at a constant speed. 
% 
% The |helperCheckScenario| function validates the seed scenario against
% these criteria. If any of the checks fail, the function returns a value
% of |0| indicating that you must modify the scenario before using it to
% generate variations variations.
% 
% The |helperPerformScenarioCollision| function modifies the scenario based
% on these conditions:
%
% * If the ego vehicle or target actor has only two waypoints, the
% |helperPerformScenarioCollision| function adds the third waypoint at the
% midpoint between them, and sets its speed value to that of the preceding
% waypoint.
% * If the ego vehicle or target actor does not travel with a constant
% speed starting from the first waypoint, the
% |helperPerformScenarioCollision| function sets their speed values to a
% constant value of 80 km/h and 10 km/h, respectively.
% * If the ego vehicle and the target actor do not collide, but have
% intersecting trajectories, |helperPerformScenarioCollision| checks if a
% collision is possible by modifying the wait time of the target actor. If
% possible, the function modifies the wait time of the target vehicle to
% create a collision event. Otherwise, the function returns an error.
%%

% Validate the seed scenario information stored in scenarioDescriptor.
checkStatus = helperCheckScenario(egoID,targetID,scenarioDescriptor);
if ~checkStatus % if scenario check failed, modify the scenario according to requirements
    scenarioDescriptor = helperPerformScenarioCollision(egoID,targetID,scenarioDescriptor,method="WaitTime");
end
%%
% To generate variants of the seed scenario, use the
% |helperGenerateScenarioVariant| function. The
% |helperGenerateScenarioVariant| function uses these input arguments: the
% |scenarioDescriptor| object, the ego vehicle ID, the target actor ID, a
% new speed for the ego vehicle, and a new collision point.

%%
% Specify the new ego speed in m/s. To convert 20 km/h to m/s, divide by
% 3.6. This results in a speed value of approximately 5.5 m/s.
egoNewSpeed = 20 * 1/3.6; % m/s
%%
% Define the new collision point, as specified in the
% <docid:driving_ug#mw_48ab91b3-fa2f-4abc-9e13-a52758090bb0 Scenario
% Variant Generation for Testing Car to Pedestrian AEB Systems> example,
% which is required for scenario variant generation. For a collision point
% at the center of the front of the ego vehicle front, specify
% |newCollisionPoint| as |0.5|.
newCollisionPoint = 0.5; 
%% 
% Generate the variant, using the |helperGenerateScenarioVariant| function,
% with the ego vehicle traveling at 20 km/h and hitting the pedestrian at
% the center of the front of the vehicle.
%%
scenarioVariant = helperGenerateScenarioVariant(scenarioDescriptor,egoID,targetID,egoNewSpeed,newCollisionPoint);
%%
% Review the generated variant scenario using the *Driving Scenario
% Designer* app.
%%
drivingScenarioDesigner(scenarioVariant);
%%
% Notice that the generated variant has an ego speed of 5.5 m/s, and the
% collision occurs at the center of the front of the ego vehicle. You can
% use this scenario with the |AEBTestBench| model to perform closed-loop
% testing.
%
% Simulink Test enables you to automate the generation and testing of
% scenario variants with the closed-loop model. Using Simulink Test, you
% can generate a large number of variations from a seed scenario and catch
% the failures.
%
%% Perform Iterative Testing with Scenario Variants
% This example includes a *Test Manager* configuration for automating
% testing of the AEB application with the generated variants. Open the
% |AEBScenarioVariationTest.mldatx| test file in the *Test Manager*.
%%
sltestmgr
testFile = sltest.testmanager.load("AEBScenarioVariationTest");
%%
%
% <<../AEB_ScenarioVariantSLTest.png>>
%
% The Simulink scripted iteration framework enables you to automate
% scenario variant generation and closed-loop testing using the test bench
% model. The iterative testing script in the
% |AEBScenarioVariationTest.mldatx| file describes how to combine scenario
% variant generation and closed-loop testing of the AEB application.
%%
% <<../ScriptedIterationCodeSnippet.png>>
% 
% The |setVariable| function sets these parameters for each iteration:
%
% * |v_set| &mdash; Set Velocity of the ego vehicle.
% * |scenario| &mdash; Driving Scenario object.
% * |egoVehDyn| &mdash; Initialization parameters for |Vehicle Dynamics|
% subsystem, specified as a structure.
%
% The |addIteration| function adds test iterations. For more information on
% creating scripted iterations, see the <docid:sltest_ug#buy138e Test
% Iterations>.
%
% After simulating the test case, the *Test Manager* uses the
% |helperPlotAEBResults| function from the *CLEANUP* callback to generate
% the post-simulation plots. For more information on these plots, see
% <docid:driving_ug#mw_b60cf0e1-2485-4e0f-8f1f-8c3300484139 Autonomous
% Emergency Braking with Sensor Fusion> example.
% 
% *Run and Explore Results for CPNC Scenario Variants*
%
%%
% To test the system-level model with the generated variants from Simulink
% Test, use this code:
% 
%   resultSet = run(testFile);
%   testFileResults = getTestFileResults(resultSet);
%   testSuiteResults = getTestSuiteResults(testFileResults);
%   testCaseResults = getTestCaseResults(testSuiteResults);
%   testIterationResults = getIterationResults(testCaseResults);
%
%%
%% Visualize Results and Review Generated Report
% The |testIterationResults| output contains the results for all the
% iterations. To plot the summary of results in the form of a color grid,
% use the |helperPlotAEBVariantResults| function. This helper function
% plots the grid results with specific colors for each cell that indicate
% the pass or fail status of closed-loop simulation of the |AEBTestBench|
% model with the generated scenario variants.
%
%%
% |helperPlotAEBVariantResults(testIterationResults,egoNewSpeeds,newCollisionPoints)|
%%
%
% <<../AEB_ScenarioVariantIterationResults.png>>
%
% In the result grid, the columns represent the collision point values of
% |0.2|, |0.3|, |0.5|, and |0.7|. The rows represent nine ego speed values
% ranging from 10 km/h to 50 km/h in steps of 5 km/h. For each value of ego
% speed, there are four possible collision points, which results in a total
% of 36 variations. The right table in the figure shows the color lookup
% table for the *VUT Test Speed* vs *VUT Impact Speed Range*. VUT test
% speed is the initial speed of the ego vehicle and VUT impact speed is the
% speed of the ego vehicle at the time of the collision. The difference
% between the test speed and the impact speed is known as speed reduction.
% This lookup table shows the speed reduction to infer AEB system
% performance at the time of the collision. The green column represents
% significant speed reduction and the red column represents lower speed
% reduction of the ego vehicle.
%
% Each cell of the grid shows the final ego speed using a color that
% corresponds to the amount of speed reduction. Notice that all cells of
% the grid are green, which indicates that the algorithm in the AEB test
% bench passed all 36 variations of the CPNC scenario.
%
% Use this code to generate the report, for further analysis of specific
% iterations:
% 
%%
%   sltest.testmanager.report(testIterationResults,"Report.pdf", ...
%   Title="Automate Testing for CPNC Scenario Variants of AEB System", ...
%   IncludeMATLABFigures=true, ...
%   IncludeErrorMessages=true, ...
%   IncludeTestResults=0, ...
%   LaunchReport=true);
%
% *Review Generated Report*
%
% Examine |Report.pdf|. Observe that the |Test environment| section
% shows the platform on which the test is run and the MATLAB version used
% for testing. The |Summary| section shows the outcome of the test and
% duration of the simulation, in seconds. The |Results| section shows pass
% or fail results based on the assessment criteria. This section also shows
% the plots logged from the |helperPlotAEBResults| function.
%%
% Enable MPC update messages.
mpcverbosity("on");