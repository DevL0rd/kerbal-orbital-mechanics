run once "libs/utils.ks".
doStartup().
run once "libs/manueverTools.ks".
parameter targetTransferBody_ARG.

transferToBody(targetTransferBody_ARG).
circularize().
doShutdown().

// main().