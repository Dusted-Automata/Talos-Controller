#pragma once
#include "types.hpp"

// code copied and adapted from https://github.com/swift-nav/libswiftnav
// should be gpl code.

#define WGS84_A 6378137.0
/** Inverse flattening of the Earth, \f$ 1/f \f$.
 * This is a defining parameter of the WGS84 ellipsoid. */
#define WGS84_IF 298.257223563
/** The flattening of the Earth, \f$ f \f$. */
#define WGS84_F (1 / WGS84_IF)
/** Semi-minor axis of the Earth in meters, \f$ b = a(1-f) \f$. */
#define WGS84_B (WGS84_A * (1 - WGS84_F))
/** Eccentricity of the Earth, \f$ e \f$ where \f$ e^2 = 2f - f^2 \f$ */
#define WGS84_E (sqrt(2 * WGS84_F - WGS84_F * WGS84_F))
#define FLOAT_EQUALITY_EPS 1e-12

static inline bool double_equal(double a, double b) { return fabs(a - b) < FLOAT_EQUALITY_EPS; }

/** Converts from WGS84 geodetic coordinates (latitude, longitude and height)
 * into WGS84 Earth Centered, Earth Fixed Cartesian (ECEF) coordinates
 * (X, Y and Z).
 *
 * Conversion from geodetic coordinates latitude, longitude and height
 * \f$(\phi, \lambda, h)\f$ into Cartesian coordinates \f$(X, Y, Z)\f$ can be
 * achieved with the following formulae:
 *
 * \f[ X = (N(\phi) + h) \cos{\phi}\cos{\lambda} \f]
 * \f[ Y = (N(\phi) + h) \cos{\phi}\sin{\lambda} \f]
 * \f[ Z = \left[(1-e^2)N(\phi) + h\right] \sin{\phi} \f]
 *
 * Where the 'radius of curvature', \f$ N(\phi) \f$, is defined as:
 *
 * \f[ N(\phi) = \frac{a}{\sqrt{1-e^2\sin^2 \phi}} \f]
 *
 * and \f$ a \f$ is the WGS84 semi-major axis and \f$ e \f$ is the WGS84
 * eccentricity. See \ref WGS84_params.
 *
 * \param llh  Geodetic coordinates to be converted, passed as
 *             [lat, lon, height] in [radians, radians, meters].
 * \param ecef Converted Cartesian coordinates are written into this array
 *             as [X, Y, Z], all in meters.
 */
// void wgsllh2ecef(const double llh[3], double ecef[3])
inline Ecef_Coord wgsllh2ecef(const double lat, const double lon, const double height = 0)
{
    Ecef_Coord ecef = {};
    double d = WGS84_E * sin(height);
    double N = WGS84_A / sqrt(1. - d * d);

    ecef.x() = (N + height) * cos(lat) * cos(lon);
    ecef.y() = (N + height) * cos(lat) * sin(lon);
    ecef.z() = ((1 - WGS84_E * WGS84_E) * N + height) * sin(lat);
    return ecef;
}

/** Converts from WGS84 Earth Centered, Earth Fixed (ECEF) Cartesian
 * coordinates (X, Y and Z) into WGS84 geodetic coordinates (latitude,
 * longitude and height).
 *
 * Conversion from Cartesian to geodetic coordinates is a much harder problem
 * than conversion from geodetic to Cartesian. There is no satisfactory closed
 * form solution but many different iterative approaches exist.
 *
 * Here we implement a relatively new algorithm due to Fukushima (2006) that is
 * very computationally efficient, not requiring any transcendental function
 * calls during iteration and very few divisions. It also exhibits cubic
 * convergence rates compared to the quadratic rate of convergence seen with
 * the more common algortihms based on the Newton-Raphson method.
 *
 * References:
 *   -# "A comparison of methods used in rectangular to Geodetic Coordinates
 *      Transformations", Burtch R. R. (2006), American Congress for Surveying
 *      and Mapping Annual Conference. Orlando, Florida.
 *   -# "Transformation from Cartesian to Geodetic Coordinates Accelerated by
 *      Halley’s Method", T. Fukushima (2006), Journal of Geodesy.
 *
 * \param ecef Cartesian coordinates to be converted, passed as [X, Y, Z],
 *             all in meters.
 * \param llh  Converted geodetic coordinates are written into this array as
 *             [lat, lon, height] in [radians, radians, meters].
 */
inline LLH wgsecef2llh(const Ecef_Coord ecef)
{
    LLH llh = {};
    /* Distance from polar axis. */
    const double p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);

    /* Compute longitude first, this can be done exactly. */
    if (!double_equal(p, 0))
    {
        llh[1] = atan2(ecef[1], ecef[0]);
    }
    else
    {
        llh[1] = 0;
    }

    /* If we are close to the pole then convergence is very slow, treat this is a
     * special case. */
    if (p < WGS84_A * 1e-16)
    {
        llh[0] = copysign(M_PI_2, ecef[2]);
        llh[2] = fabs(ecef[2]) - WGS84_B;
        return llh;
    }

    /* Caluclate some other constants as defined in the Fukushima paper. */
    const double P = p / WGS84_A;
    const double e_c = sqrt(1. - WGS84_E * WGS84_E);
    const double Z = fabs(ecef[2]) * e_c / WGS84_A;

    /* Initial values for S and C correspond to a zero height solution. */
    double S = Z;
    double C = e_c * P;

    /* Neither S nor C can be negative on the first iteration so
     * starting prev = -1 will not cause and early exit. */
    double prev_C = -1;
    double prev_S = -1;

    double A_n, B_n, D_n, F_n;

    /* Iterate a maximum of 10 times. This should be way more than enough for all
     * sane inputs */
    for (int i = 0; i < 10; i++)
    {
        /* Calculate some intermediate variables used in the update step based on
         * the current state. */
        A_n = sqrt(S * S + C * C);
        D_n = Z * A_n * A_n * A_n + WGS84_E * WGS84_E * S * S * S;
        F_n = P * A_n * A_n * A_n - WGS84_E * WGS84_E * C * C * C;
        B_n = 1.5 * WGS84_E * S * C * C * (A_n * (P * S - Z * C) - WGS84_E * S * C);

        /* Update step. */
        S = D_n * F_n - B_n * S;
        C = F_n * F_n - B_n * C;

        /* The original algorithm as presented in the paper by Fukushima has a
         * problem with numerical stability. S and C can grow very large or small
         * and over or underflow a double. In the paper this is acknowledged and
         * the proposed resolution is to non-dimensionalise the equations for S and
         * C. However, this does not completely solve the problem. The author caps
         * the solution to only a couple of iterations and in this period over or
         * underflow is unlikely but as we require a bit more precision and hence
         * more iterations so this is still a concern for us.
         *
         * As the only thing that is important is the ratio T = S/C, my solution is
         * to divide both S and C by either S or C. The scaling is chosen such that
         * one of S or C is scaled to unity whilst the other is scaled to a value
         * less than one. By dividing by the larger of S or C we ensure that we do
         * not divide by zero as only one of S or C should ever be zero.
         *
         * This incurs an extra division each iteration which the author was
         * explicityl trying to avoid and it may be that this solution is just
         * reverting back to the method of iterating on T directly, perhaps this
         * bears more thought?
         */

        if (S > C)
        {
            C = C / S;
            S = 1;
        }
        else
        {
            S = S / C;
            C = 1;
        }

        /* Check for convergence and exit early if we have converged. */
        if (fabs(S - prev_S) < 1e-16 && fabs(C - prev_C) < 1e-16)
        {
            break;
        }
        prev_S = S;
        prev_C = C;
    }

    A_n = sqrt(S * S + C * C);
    llh[0] = copysign(1.0, ecef[2]) * atan(S / (e_c * C));
    llh[2] =
        (p * e_c * C + fabs(ecef[2]) * S - WGS84_A * e_c * A_n) / sqrt(e_c * e_c * C * C + S * S);
    return llh;
}

/** Populates a provided 3x3 matrix with the appropriate rotation
 * matrix to transform from ECEF to NED coordinates, given the
 * provided LLH reference vector.
 *
 * \param llh  Latitude longitude and height of the reference location
 *             according to the WGS ellipsoid.  latitude and longitude
 *             are assumed to be in radians, height in meters.
 * \param M        3x3 matrix to be populated with rotation matrix.
 */
inline Eigen::Matrix3d wgs_ecef2ned_matrix(const LLH llh)
{
    double sin_lat = sin(llh[0]), cos_lat = cos(llh[0]), sin_lon = sin(llh[1]),
           cos_lon = cos(llh[1]);
    Eigen::Matrix3d M;
    M(0, 0) = -sin_lat * cos_lon;
    M(0, 1) = -sin_lat * sin_lon;
    M(0, 2) = cos_lat;
    M(1, 0) = -sin_lon;
    M(1, 1) = cos_lon;
    M(1, 2) = 0.0;
    M(2, 0) = -cos_lat * cos_lon;
    M(2, 1) = -cos_lat * sin_lon;
    M(2, 2) = -sin_lat;
    return M;
}

/** Populates a provided 3x3 matrix with the appropriate rotation
 * matrix to transform from ECEF to NED coordinates, given the
 * provided ECEF reference vector.
 *
 * \param ref_ecef Cartesian coordinates of reference vector, passed as
 *                 [X, Y, Z], all in meters.
 * \param M        3x3 matrix to be populated with rotation matrix.
 */
inline Eigen::Matrix3d ecef2ned_matrix(const Ecef_Coord ref_ecef)
{
    LLH llh = wgsecef2llh(ref_ecef);
    Eigen::Matrix3d M = wgs_ecef2ned_matrix(llh);
    return M;
}

/** Converts a vector in WGS84 Earth Centered, Earth Fixed (ECEF) Cartesian
 * coordinates to the local North, East, Down (NED) frame of a reference point,
 * also given in WGS84 ECEF coordinates.
 *
 * Note, this function only \e rotates the ECEF vector into the NED frame of
 * the reference point, as would be appropriate for e.g. a velocity vector. To
 * determine the distance between the point and the reference point in the NED
 * frame of the reference point, see \ref wgsecef2ned_d.
 *
 * \see wgsecef2ned_d.
 *
 * \param ecef      Cartesian coordinates of the point, passed as [X, Y, Z],
 *                  all in meters.
 * \param ref_ecef  Cartesian coordinates of the reference point, passed as
 *                  [X, Y, Z], all in meters.
 * \param ned       The North, East, Down vector is written into this array as
 *                  [N, E, D], all in meters.
 */
inline Eigen::Vector3d wgsecef2ned(const Ecef_Coord ecef, const Ecef_Coord ref_ecef)
{
    // double M[3][3];
    // ecef2ned_matrix(ref_ecef, M);
    // matrix_multiply(3, 3, 1, (double *)M, ecef, ned);

    Eigen::Matrix3d M = ecef2ned_matrix(ref_ecef);
    Eigen::Vector3d ned = M * ecef;
    return ned;
}

/** Returns the vector \e to a point given in WGS84 Earth Centered, Earth Fixed
 * (ECEF) Cartesian coordinates \e from a reference point, also given in WGS84
 * ECEF coordinates, in the local North, East, Down (NED) frame of the
 * reference point.
 *
 * \see wgsecef2ned.
 *
 * \param ecef      Cartesian coordinates of the point, passed as [X, Y, Z],
 *                  all in meters.
 * \param ref_ecef  Cartesian coordinates of the reference point, passed as
 *                  [X, Y, Z], all in meters.
 * \param ned       The North, East, Down vector is written into this array as
 *                  [N, E, D], all in meters.
 */
inline Eigen::Vector3d wgsecef2ned_dist(const Ecef_Coord ecef, const Ecef_Coord ref_ecef)
{
    Eigen::Vector3d new_ecef = ecef - ref_ecef;
    Eigen::Vector3d ned = wgsecef2ned(new_ecef, ref_ecef);
    return ned
}

/** Converts a vector in local North, East, Down (NED) coordinates relative to
 * a reference point given in WGS84 Earth Centered, Earth Fixed (ECEF) Cartesian
 * coordinates to a vector in WGS84 ECEF coordinates.
 *
 * Note, this function only \e rotates the NED vector into the ECEF frame, as
 * would be appropriate for e.g. a velocity vector. To pass an NED position in
 * the reference frame of the NED, see \ref wgsned2ecef_d.
 *
 * \see wgsned2ecef_d.
 *
 * \param ned       The North, East, Down vector is passed as [N, E, D], all in
 *                  meters.
 * \param ref_ecef  Cartesian coordinates of the reference point, passed as
 *                  [X, Y, Z], all in meters.
 * \param ecef      Cartesian coordinates of the point written into this array,
 *                  [X, Y, Z], all in meters.
 */
inline Ecef_Coord wgsned2ecef(const Eigen::Vector3d ned, const Ecef_Coord ref_ecef)
{
    Eigen::Matrix3d M = ecef2ned_matrix(ref_ecef);
    Ecef_Coord ecef = M.transpose() * ned;
    return ecef;
}

/** For a point given in the local North, East, Down (NED) frame of a provided
 * ECEF reference point, return the vector to that point in ECEF coordinates.
 *
 * Intended for calculating a new ECEF position. For converting velocity
 * vectors,
 * \see wgsned2ecef.
 *
 * \param ned       The North, East, Down vector is passed as [N, E, D], all in
 *                  meters.
 * \param ref_ecef  Cartesian coordinates of the reference point, passed as
 *                  [X, Y, Z], all in meters.
 * \param ecef      Cartesian coordinates of the point written into this array,
 *                  [X, Y, Z], all in meters.
 */
inline Ecef_Coord wgsned2ecef_d(const Eigen::Vector3d ned, const Ecef_Coord ref_ecef)
{
    Ecef_Coord ecef = wgsned2ecef(ned, ref_ecef);
    ecef = ecef + ref_ecef;
    return ecef;
}
