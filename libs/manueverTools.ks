function ternarySearch {
  parameter f, left, right, absolutePrecision.
  until false {
    if abs(right - left) < absolutePrecision {
      return (left + right) / 2.
    }
    local leftThird is left + (right - left) / 3.
    local rightThird is right - (right - left) / 3.
    if f(leftThird) < f(rightThird) {
      set left to leftThird.
    } else {
      set right to rightThird.
    }
  }
}

function protectFromPast {
  parameter originalFunction.
  local replacementFunction is {
    parameter data.
    if data[0] < time:seconds + 15 {
      return 2^64.
    } else {
      return originalFunction(data).
    }
  }.
  return replacementFunction@.
}


function improveConverge {
  parameter data, scoreFunction.
  for stepSize in list(100, 50, 10, 1, 0.1) {
    until false {
      local oldScore is scoreFunction(data).
      set data to improve(data, stepSize, scoreFunction).
      if oldScore <= scoreFunction(data) {
        break.
      }
    }
  }
  return data.
}


function improve {
  parameter data, stepSize, scoreFunction.
  local scoreToBeat is scoreFunction(data).
  local bestCandidate is data.
  local candidates is list().
  local index is 0.
  until index >= data:length {
    local incCandidate is data:copy().
    local decCandidate is data:copy().
    set incCandidate[index] to incCandidate[index] + stepSize.
    set decCandidate[index] to decCandidate[index] - stepSize.
    candidates:add(incCandidate).
    candidates:add(decCandidate).
    set index to index + 1.
  }
  for candidate in candidates {
    local candidateScore is scoreFunction(candidate).
    if candidateScore < scoreToBeat {
      set scoreToBeat to candidateScore.
      set bestCandidate to candidate.
    }
  }
  return bestCandidate.
}

function lockSteeringAtManeuverTarget {
  parameter mnv.
  lock steering to mnv:burnvector.
}


function stoppingDistance {
  local grav is constant():g * (body:mass / body:radius^2).
  local maxDeceleration is (ship:availableThrust / ship:mass) - grav.
  return ship:verticalSpeed^2 / (2 * maxDeceleration).
}


function distanceToGround {
  return altitude - body:geopositionOf(ship:position):terrainHeight - 4.7.
}


function groundSlope {
  local east is vectorCrossProduct(north:vector, up:vector).

  local center is ship:position.

  local a is body:geopositionOf(center + 5 * north:vector).
  local b is body:geopositionOf(center - 3 * north:vector + 4 * east).
  local c is body:geopositionOf(center - 3 * north:vector - 4 * east).

  local a_vec is a:altitudePosition(a:terrainHeight).
  local b_vec is b:altitudePosition(b:terrainHeight).
  local c_vec is c:altitudePosition(c:terrainHeight).

  return vectorCrossProduct(c_vec - a_vec, b_vec - a_vec):normalized.
}

local originalVector to -1.
function isManeuverComplete {
  parameter mnv.
  if vang(originalVector, mnv:burnvector) > 90 {
    return true.
  }
  return false.
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


function calculateStartTime {
  parameter mnv.
  return time:seconds + mnv:eta - maneuverBurnTime(mnv) / 2.
}


function removeManeuverFromFlightPlan {
  parameter mnv.
  remove mnv.
}

function addManeuverToFlightPlan {
  parameter mnv.
  add mnv.
}

function executeManeuver {
  parameter mList.
  local mnv is node(mList[0], mList[1], mList[2], mList[3]).
  addManeuverToFlightPlan(mnv).
  local startTime is calculateStartTime(mnv).
  print "Warping to maneuver...".
  warpto(startTime - 15).
  print "Positioning and waiting to burn.".
  lockSteeringAtManeuverTarget(mnv).
  wait until time:seconds > startTime.
  print "Executing maneuver.".
  lock throttle to 1.
  set originalVector to mnv:burnvector.
  wait until isManeuverComplete(mnv).
  lock throttle to 0.
  unlock steering.
  removeManeuverFromFlightPlan(mnv).
}


function circularize {
    print "Circularizing orbit...".

    lock desired_obt_speed  to sqrt(ship:body:Mu / (ship:altitude + ship:body:radius)).
    lock desired_obt_vector to vxcl(-ship:body:position, ship:velocity:orbit):Normalized.
    lock desired_obt_vel    to (desired_obt_vector * desired_obt_speed).
    lock correction_vector  to (desired_obt_vel - ship:velocity:orbit).
    lock steering to correction_vector.
    if eta:apoapsis < eta:periapsis {
        print "Warping to apoapsis...".
        if Eta:Apoapsis > 0 {
          warpTo(Time:Seconds + Eta:Apoapsis - 15).
          wait Eta:Apoapsis - 2.
        }
    } else {
        print "Warping to periapsis...".
        warpTo(Time:Seconds + Eta:Periapsis - 15).
        wait Eta:Periapsis - 2.
    }
    

    print "Firing engines to circularize orbit.".
    until (correction_vector:MAG < 0.01)
    {
        if vang(correction_vector, ship:facing:forevector) < 2 {
            wait until maxThrust > 0.
            local correction_factor is min(1, correction_vector:MAG * ship:mass / ship:maxthrust).
            lock throttle to correction_factor.
        } else {
            lock throttle to 0.
        }
    }

    lock throttle to 0.
    unlock steering.
    unlock correction_vector.
    unlock desired_obt_vel.
    unlock desired_obt_vector.
    unlock desired_obt_speed.
}

// function eccentricityScore {
//   parameter data.
//   local mnv is node(time:seconds + eta:apoapsis, 0, 0, data[0]).
//   addManeuverToFlightPlan(mnv).
//   local result is mnv:orbit:eccentricity.
//   removeManeuverFromFlightPlan(mnv).
//   return result.
// }

// function circularize {
//   local circ is list(0).
//   set circ to improveConverge(circ, eccentricityScore@).
//   wait until altitude > 70000.
//   executeManeuver(list(time:seconds + eta:apoapsis, 0, 0, circ[0])).
// }







// local minimumThrustPrecision is 0.005.
// local warpLeadTime is 30.
// function increase_period {
//     parameter target_period_minutes.
//     set targetPeriodHours to target_period_minutes / 60.
//     lock currentPeriodHours to orbit:period / 60 / 60.
//     set targetPeriod to target_period_minutes * 60.

//     print "Warping to apoapsis...".
//     warpTo(Time:Seconds + Eta:Apoapsis - warpLeadTime).
//     print "Locking steering to prograde, and waiting to burn.".
//     lock steering to prograde.
//     wait Eta:Apoapsis - 2.

//     print "Burning to increase orbital period...".
//     lock throttle to max(minimumThrustPrecision, min(1, targetPeriodHours - currentPeriodHours)).

    
//     local timeOfLastPrint is Time:Seconds - 1.
//     until targetPeriod - orbit:period <= 0.05 {
//         if Time:Seconds - timeOfLastPrint >= 1 {
//             print("Throt: " + throttle + " | " + "Period: " + orbit:period / 60 + "m").
//             set timeOfLastPrint to Time:Seconds.
//         }
//     }
//     lock throttle to 0.
//     unlock currentPeriodHours.
// }


// function decrease_period {
//     parameter target_period_minutes.
//     set targetPeriodHours to target_period_minutes / 60.
//     lock currentPeriodHours to orbit:period / 60 / 60.
//     set targetPeriod to target_period_minutes * 60.

//     // time until apoapsis
//     print "Warping to periapsis...".
//     warpTo(Time:Seconds + Eta:Periapsis - warpLeadTime).
//     print "Locking steering to retrograde, and waiting to burn.".
//     lock steering to retrograde.
//     wait Eta:Periapsis - 2.

//     print "Burning to decrease orbital period...".
//     lock throttle to max(minimumThrustPrecision, min(1, currentPeriodHours - targetPeriodHours)).

//     local timeOfLastPrint is Time:Seconds - 5.
//     until orbit:period - targetPeriod <= 0.05 {
//         if Time:Seconds - timeOfLastPrint >= 5 {
//             print("Throt: " + throttle + " | " + "Period: " + orbit:period / 60 + "m").
//             set timeOfLastPrint to Time:Seconds.
//         }
//     }
//     print("Throt: " + throttle + " | " + "Period: " + orbit:period / 60 + "m").
//     lock throttle to 0.
//     unlock currentPeriodHours.
// }


local targetPeriod is 0.
function increase_periodScore {
  parameter data.
  local mnv is node(time:seconds + eta:apoapsis, 0, 0, data[3]). // ignore axis to prevent wasted time.
  addManeuverToFlightPlan(mnv).
  local result is abs(targetPeriod - mnv:orbit:period).
  removeManeuverFromFlightPlan(mnv).
  return result.
}

function decrease_periodScore {
  parameter data.
  local mnv is node(time:seconds + eta:periapsis, 0, 0, data[3]). // ignore axis to prevent wasted time.
  addManeuverToFlightPlan(mnv).
  local result is abs(targetPeriod - mnv:orbit:period).
  removeManeuverFromFlightPlan(mnv).
  return result.
}

function increase_period {
  parameter target_period_minutes.
  set targetPeriod to target_period_minutes * 60.
  local mnv is list(0, 0, 0, 0).
  set mnv to improveConverge(mnv, increase_periodScore@).
  executeManeuver(list(time:seconds + eta:apoapsis, 0, 0, mnv[3])).
}

function increase_period_circularly {
    parameter target_period_minutes.
    local period_minutes is orbit:period / 60.
    local difference is target_period_minutes - period_minutes.
    local half_difference is difference / 2.
    // local current_period_minutes is orbit:period / 60.
    increase_period(period_minutes + half_difference).
    wait 1.
    increase_period(target_period_minutes).
}

function decrease_period {
  parameter target_period_minutes.
  set targetPeriod to target_period_minutes * 60.
  local mnv is list(time:seconds + eta:periapsis, 0, 0, 0).
  set mnv to improveConverge(mnv, decrease_periodScore@).
  executeManeuver(list(time:seconds + eta:periapsis, 0, 0, mnv[3])).
}

function decrease_period_circularly {
    parameter target_period_minutes.
    local period_minutes is orbit:period / 60.
    local difference is period_minutes - target_period_minutes.
    local half_difference is difference / 2.
    decrease_period(period_minutes - half_difference).
    wait 1.
    decrease_period(target_period_minutes).
}


function orbitAtPeriod {
    parameter target_period_minutes.
    parameter isCircular.
    local period_minutes is orbit:period / 60.
    local difference is abs(period_minutes - target_period_minutes).
    if period_minutes < target_period_minutes {
        if isCircular {
            print "Increasing period circularly by " + round(difference) + " minutes, to " + target_period_minutes + ".".
            increase_period_circularly(target_period_minutes).
        } else {
            print "Increasing period eliptically by " + round(difference) + " minutes, to " + target_period_minutes + ".".
            increase_period(target_period_minutes).
        }
    } else {
        if isCircular {
            print "Decreasing period circularly by " + round(difference) + " minutes, to " + target_period_minutes + ".".
            decrease_period_circularly(target_period_minutes).
        } else {
            print "Decreasing period eliptically by " + round(difference) + " minutes, to " + target_period_minutes + ".".
            decrease_period(target_period_minutes).
        }
    }
}






function altitudeAt {
  parameter t.
  // return Kerbin:altitudeOf(positionAt(ship, t)).
  return body:altitudeOf(positionAt(ship, t)).
}
local targetTransferBody is mun.
function distanceToBodyAtApoapsis {
  parameter mnv.
  local apoapsisTime is ternarySearch(
    altitudeAt@, 
    time:seconds + mnv:eta, 
    time:seconds + mnv:eta + (mnv:orbit:period / 2),
    1
  ).
  return (positionAt(ship, apoapsisTime) - positionAt(targetTransferBody, apoapsisTime)):mag.
}

function bodyTransferScore {
  parameter data.
  local mnv is node(data[0], data[1], data[2], data[3]).
  addManeuverToFlightPlan(mnv).
  local result is 0.
  if mnv:orbit:hasNextPatch {
    if mnv:orbit:nextPatch:periapsis < 200000 or mnv:orbit:nextPatch:body <> targetTransferBody {
      set result to 999999999999999.
    } else {
      set result to mnv:orbit:nextPatch:periapsis.
    }
  } else {
    set result to distanceToBodyAtApoapsis(mnv).
  }
  removeManeuverFromFlightPlan(mnv).
  return result.
}

function angleToBody {
  parameter t.
  return vectorAngle(
    body:position - positionAt(ship, t),
    body:position - positionAt(targetTransferBody, t)
  ).
}

function transferToBody {
  parameter targetBody.
  set targetTransferBody to targetBody.
  print "Calculating transfer to " + targetBody:name + ".".
  local startSearchTime is ternarySearch(
    angleToBody@,
    time:seconds + 30, 
    time:seconds + 30 + orbit:period,
    1
  ).
  local transfer is list(startSearchTime, 0, 0, 0).
  set transfer to improveConverge(transfer, protectFromPast(bodyTransferScore@)).

  executeManeuver(transfer).
  wait 1.
  warpto(time:seconds + obt:nextPatchEta - 5).
  wait until body = targetBody.
}

function deOrbit {
    print "De-Orbiting...".
    lock steering to retrograde.
    wait 5.
    if body:atm:exists {
        print "Atmosphere detected. Burning to enter atmosphere.".
        lock throttle to 1.
        wait until periapsis <= body:atm:height - 2500.
        lock throttle to 0.
        print "Waiting for atmosphere...".
        set warp to 3.
        wait until altitude <= body:atm:height + 500.
        set warp to 0.
        wait until altitude <= body:atm:height.
    } else {
        print "Burning to de-orbit".
        lock throttle to 1.
        wait until periapsis <= -body:radius * 0.9.
        lock throttle to 0.
    }
    print "De-Orbit complete.".
}

function land {
  lock steering to srfRetrograde.
  local oldRcs is rcs.
  set rcs to true.
  print periapsis.
  if periapsis > 0 {
    deOrbit().
  }
  lock pct to stoppingDistance() / distanceToGround().
  lock throttle to pct.
  print "Landing...".
  wait 1.
  set warp to 4.
  wait until pct > 0.1.
  set warp to 3.
  wait until pct > 0.2.
  set warp to 0.
  wait until pct > 1.
  when distanceToGround() < 500 then { gear on. }
  wait until ship:verticalSpeed > 0.
  lock throttle to 0.
  lock steering to groundSlope().
  wait 30.
  unlock steering.
  set rcs to oldRcs.
}
