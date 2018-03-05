#include <stdio.h>
#include <stdlib.h>

#include "dfs.h"

double
dfs_rho (double temperature, double salinity);


double
dfs_simple (double rho, double p_gage)
{
    // p_gage = rho*g*h
    return p_gage / (DFS_GRAVITY * rho);
}
