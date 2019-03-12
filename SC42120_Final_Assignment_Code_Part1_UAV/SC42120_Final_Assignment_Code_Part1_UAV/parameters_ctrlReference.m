%% Control surfaces deflections

Ctr.Gen.stepTimeAil = 10;
Ctr.Gen.stepTimeAilFinal = Ctr.Gen.stepTimeAil + 5;
Ctr.Gen.stepAilDelay = 10;
Ctr.Gen.stepAil = 10;                      % Aileron deflection [deg] - positive yawing rightward

Ctr.Gen.stepTimeElev = 10;
Ctr.Gen.stepTimeElevFinal = Ctr.Gen.stepTimeElev + 60;
Ctr.Gen.stepElev = 0;                       % Elevator deflection [deg] - positive pitching downward

Ctr.Gen.stepTimeRudd = 100;
Ctr.Gen.stepTimeRuddFinal = Ctr.Gen.stepTimeRudd + 2;
Ctr.Gen.stepAilDelay = 10;

Ctr.Gen.stepRudd = 0;                       % Rudder deflection [deg] - positive tail left