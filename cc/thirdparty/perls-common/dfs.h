#ifndef __PERLS_COMMON_DFS_H__
#define __PERLS_COMMON_DFS_H__

#define DFS_RHO_FRESHWATER (998.0)    // kg/m^3
#define DFS_RHO_SALTWATER  (1027.0)   // kg/m^3
#define DFS_GRAVITY        (9.81)     // m/s^2
#define DFS_P_ATM_NOMINAL  (101325.0) // 101325 Pa = 1 atm

#ifdef __cplusplus
extern "C" {
#endif

/* returns water density rho based upon measured temperature and salinity
 * temperature - Celsius
 * salinity - ppt (0-freshwater, ~35-saltwater)
 * rho - kg/m^3
 */
double
dfs_rho (double temperature, double salinity);


/* returns depth H using the simple formulation p_gage = rho*g*H
 * p_gage - pressure in Pascals
 * rho    - density of water in kg/m^3
 * g      - 9.81 m/s^2
 * H      - depth in meters
 */
double
dfs_simple (double rho, double p_gage);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_COMMON_DFS_H__
