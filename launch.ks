run once "libs/utils.ks".
run once "libs/manueverTools.ks".
parameter targetTransferBody_ARG is mun.
parameter shouldLand_ARG is true.

set targetdirection to heading(90,90).
lock steering to targetdirection.
doStartup().
lock throttle to 0.7.

local waitTime is 5.

until waitTime < 1 {
    print "Launching in " + waitTime + " seconds.".
    wait 1.
    set waitTime to waitTime - 1.
}
print "Launch!".
doSafeStage().

//wait until out of atmosphere
print "Waiting to leave atmosphere...".
wait until altitude > 70000.
lock throttle to 0.
wait 1.

print "Tilting to 5 degrees.".
set rcs to true.
set targetdirection to heading(90,5).
lock steering to targetdirection.

circularize().
if targetTransferBody_ARG <> body {
    transferToBody(targetTransferBody_ARG).
    circularize().
}
if shouldLand_ARG {
    doSafeStage().
    wait 1.
    
    doSafeStage().
    wait 1.
    land().
}
doShutdown().