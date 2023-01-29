
function doSafeStage {
  wait until stage:ready.
  print "Staging.".
  stage.
}

function needToStage {
  list engines in es.
  for e in es if e:ignition and e:flameout { return true.}
}


function doStartup {
    lock throttle to 0.
    set sas to false.
    set rcs to false.
    when needToStage then {
      doSafeStage.
      if not ship:status = "PRELAUNCH" and availableThrust = 0 {
        return.
      }
      preserve.
    }
}

function maneuverBurnTime {
  parameter mnv.
  local dV is mnv:deltaV:mag.
  local g0 is 9.80665.
  local isp is 0.

  list engines in myEngines.
  for en in myEngines {
    if en:ignition and not en:flameout {
      set isp to isp + (en:isp * (en:availableThrust / ship:availableThrust)).
    }
  }

  local mf is ship:mass / constant():e^(dV / (isp * g0)).
  local fuelFlow is ship:availableThrust / (isp * g0).
  local t is (ship:mass - mf) / fuelFlow.

  return t.
}

function doShutdown {
    lock throttle to 0.
    set sas to true.
    set rcs to false.
}
