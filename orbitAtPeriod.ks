parameter target_period_m_ARG, isCircular_ARG, useRcs_ARG.
run once "libs/utils.ks".
run once "libs/manueverTools.ks".
doStartup().
when maxThrust = 0 then {
    stage.
    preserve.
}
set rcs to useRcs_ARG.
orbitAtPeriod(target_period_m_ARG, isCircular_ARG).
doShutdown().