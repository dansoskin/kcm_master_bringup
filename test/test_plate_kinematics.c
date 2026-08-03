/* Host test for plate_kinematics. Pure math, no hardware.
 *   gcc -I../Core/Inc test_plate_kinematics.c ../Core/Src/plate_kinematics.c -lm
 * See test/run.sh */
#include <stdio.h>
#include <math.h>
#include "plate_kinematics.h"

static int failures = 0;
static int checks   = 0;

#define DEG (3.14159265358979323846 / 180.0)

static void check(int cond, const char *what)
{
    checks++;
    if (!cond) { failures++; printf("  FAIL: %s\n", what); }
}

static void check_near(float got, float want, float tol, const char *what)
{
    checks++;
    if (!(fabsf(got - want) <= tol)) {
        failures++;
        printf("  FAIL: %s (got %.6f, want %.6f)\n", what, got, want);
    }
}

/* Independent reference: build the plate normal, then measure how far it leans
 * from vertical. Deliberately not the implementation's formula -- this checks
 * the resulting pose, not the arithmetic path taken to get there. */
static double tilt_of(double az_deg, double ti_deg)
{
    double nx = sin(ti_deg * DEG) * cos(az_deg * DEG);
    double ny = sin(ti_deg * DEG) * sin(az_deg * DEG);
    double nz = cos(ti_deg * DEG);
    return atan2(sqrt(nx * nx + ny * ny), nz) / DEG;
}

/* Double-precision reference for the two motor angles, restated from the plate
 * normal. Used as the thing to differentiate when checking the analytic rates. */
static double ref_x(double az_deg, double ti_deg)
{
    return atan2(sin(ti_deg * DEG) * cos(az_deg * DEG), cos(ti_deg * DEG)) / DEG;
}
static double ref_y(double az_deg, double ti_deg)
{
    return atan2(-sin(ti_deg * DEG) * sin(az_deg * DEG), cos(ti_deg * DEG)) / DEG;
}

/* Recover (azimuth, tilt) from the two motor angles. */
static void inverse(float x_deg, float y_deg, double *az, double *ti)
{
    double tx = tan(x_deg * DEG);            /* = tan(theta) cos(phi) */
    double ty = -tan(y_deg * DEG);           /* = tan(theta) sin(phi) */
    *az = atan2(ty, tx) / DEG;
    if (*az < 0) *az += 360.0;
    *ti = atan(sqrt(tx * tx + ty * ty)) / DEG;
}

int main(void)
{
    float x, y;
    plate_status_t st;

    printf("level is level regardless of azimuth\n");
    for (float az = 0; az < 360.0f; az += 37.0f) {
        x = y = 12345.0f;
        st = plate_angles_from_tilt(az, 0.0f, &x, &y);
        check(st == PLATE_OK, "tilt 0 accepted");
        check_near(x, 0.0f, 1e-6f, "tilt 0 -> x is 0");
        check_near(y, 0.0f, 1e-6f, "tilt 0 -> y is 0");
    }

    printf("pure +X lean puts everything on the toward-X motor\n");
    st = plate_angles_from_tilt(0.0f, 10.0f, &x, &y);
    check(st == PLATE_OK, "(0,10) accepted");
    check_near(x, 10.0f, 1e-4f, "(0,10) -> x is +10");
    check_near(y,  0.0f, 1e-4f, "(0,10) -> y is 0");

    printf("pure +Y lean puts everything on the toward-Y motor\n");
    st = plate_angles_from_tilt(90.0f, 10.0f, &x, &y);
    check(st == PLATE_OK, "(90,10) accepted");
    check_near(x,   0.0f, 1e-4f, "(90,10) -> x is 0");
    check_near(y, -10.0f, 1e-4f, "(90,10) -> y is -10 (right-hand rule)");

    printf("45 deg azimuth splits evenly between the two motors\n");
    plate_angles_from_tilt(45.0f, 10.0f, &x, &y);
    check(fabsf(x) > 1e-3f && fabsf(y) > 1e-3f, "(45,10) -> both motors move");
    check_near(fabsf(x), fabsf(y), 1e-4f, "(45,10) -> equal magnitudes");

    printf("signs are correct in all four quadrants\n");
    plate_angles_from_tilt( 45.0f, 10.0f, &x, &y);
    check(x > 0 && y < 0, "az 45  -> x>0, y<0");
    plate_angles_from_tilt(135.0f, 10.0f, &x, &y);
    check(x < 0 && y < 0, "az 135 -> x<0, y<0");
    plate_angles_from_tilt(225.0f, 10.0f, &x, &y);
    check(x < 0 && y > 0, "az 225 -> x<0, y>0");
    plate_angles_from_tilt(315.0f, 10.0f, &x, &y);
    check(x > 0 && y > 0, "az 315 -> x>0, y>0");

    printf("azimuth wraps\n");
    float wx, wy;
    plate_angles_from_tilt( 90.0f, 10.0f, &x,  &y);
    plate_angles_from_tilt(450.0f, 10.0f, &wx, &wy);
    check_near(wx, x, 1e-4f, "az 450 == az 90 (x)");
    check_near(wy, y, 1e-4f, "az 450 == az 90 (y)");

    printf("round-trip across the whole working range\n");
    int rt_bad = 0, rt_n = 0;
    for (double az = 0; az < 360.0; az += 7.0) {
        for (double ti = 0.5; ti <= 30.0; ti += 0.5) {
            if (plate_angles_from_tilt((float)az, (float)ti, &x, &y) != PLATE_OK) {
                rt_bad++; continue;
            }
            double raz, rti;
            inverse(x, y, &raz, &rti);
            double daz = fabs(raz - az);
            if (daz > 180.0) daz = 360.0 - daz;
            if (daz > 0.01 || fabs(rti - ti) > 0.01) rt_bad++;
            /* and the commanded pose really is that far from level */
            if (fabs(tilt_of(az, ti) - ti) > 1e-6) rt_bad++;
            rt_n++;
        }
    }
    printf("  round-trip: n=%d mismatches=%d\n", rt_n, rt_bad);
    check(rt_n > 0 && rt_bad == 0, "round-trip exact over az 0-360, tilt 0.5-30");

    printf("out-of-range tilt is refused, outputs untouched\n");
    x = y = 999.0f;
    check(plate_angles_from_tilt(0.0f,  31.0f, &x, &y) == PLATE_ERR_RANGE, "tilt 31 refused");
    check(plate_angles_from_tilt(0.0f, 100.0f, &x, &y) == PLATE_ERR_RANGE, "tilt 100 refused");
    check(plate_angles_from_tilt(0.0f, -31.0f, &x, &y) == PLATE_ERR_RANGE, "tilt -31 refused");
    check_near(x, 999.0f, 0.0f, "refused tilt leaves x untouched");
    check_near(y, 999.0f, 0.0f, "refused tilt leaves y untouched");
    check(plate_angles_from_tilt(0.0f, 30.0f, &x, &y) == PLATE_OK, "tilt 30 exactly is allowed");

    printf("bad arguments are refused, outputs untouched\n");
    x = y = 777.0f;
    check(plate_angles_from_tilt(NAN, 10.0f, &x, &y) == PLATE_ERR_BAD_ARG, "NaN azimuth refused");
    check(plate_angles_from_tilt(0.0f, NAN,  &x, &y) == PLATE_ERR_BAD_ARG, "NaN tilt refused");
    check(plate_angles_from_tilt(INFINITY, 10.0f, &x, &y) == PLATE_ERR_BAD_ARG, "Inf azimuth refused");
    check(plate_angles_from_tilt(0.0f, INFINITY, &x, &y) == PLATE_ERR_BAD_ARG, "Inf tilt refused");
    check_near(x, 777.0f, 0.0f, "bad arg leaves x untouched");
    check_near(y, 777.0f, 0.0f, "bad arg leaves y untouched");
    check(plate_angles_from_tilt(0.0f, 10.0f, NULL, &y) == PLATE_ERR_BAD_ARG, "NULL out_x refused");
    check(plate_angles_from_tilt(0.0f, 10.0f, &x, NULL) == PLATE_ERR_BAD_ARG, "NULL out_y refused");

    /* ---------------- plate_rates_from_spin ---------------- */

    printf("rates match a numeric derivative of the geometry, in BOTH variables\n");
    {
        /* Differentiate the DOUBLE reference (ref_x/ref_y below), not the float32
         * implementation. Differencing float32 angles has a ~0.03 deg/s noise
         * floor of its own -- the same order as the error being looked for, so
         * it cannot resolve a real mistake. The reference restates the geometry
         * from the plate normal and never uses the derivative formula, so this
         * stays an independent check of the derivation.
         *
         * The rate is a TOTAL derivative now, so the reference is the sum of
         * both partials. Cases with r != 0 are what catch a missing or
         * mis-signed tilt term; the azimuth-only rows (r == 0) keep the
         * original coverage intact. */
        const double h = 1e-6;      /* deg, used for both partials */
        const double wr[][2] = {
            {  90.0,   0.0 },   /* pure spin, the original cases            */
            { -37.5,   0.0 },
            { 360.0,   0.0 },
            {   0.0,   5.0 },   /* pure tilt ramp, opening                  */
            {   0.0,  -5.0 },   /* pure tilt ramp, closing                  */
            {  90.0,   5.0 },   /* the real case: spinning while tilting in */
            { -90.0,  12.0 },
            { 360.0,  -3.0 },
        };
        int n = 0;
        double worst_rel = 0.0, worst_abs = 0.0;
        for (double az = 0; az < 360.0; az += 3.0) {
            for (double ti = 1.0; ti <= 30.0; ti += 1.0) {
                for (size_t s = 0; s < sizeof(wr) / sizeof(wr[0]); s++) {
                    const float w = (float)wr[s][0];
                    const float r = (float)wr[s][1];
                    float rx, ry;
                    if (plate_rates_from_spin((float)az, (float)ti, w, r, &rx, &ry) != PLATE_OK) {
                        worst_abs = 1e9; continue;
                    }
                    /* total derivative = dref/daz * w + dref/dti * r */
                    double nx = (ref_x(az + h, ti) - ref_x(az - h, ti)) / (2.0 * h) * w
                              + (ref_x(az, ti + h) - ref_x(az, ti - h)) / (2.0 * h) * r;
                    double ny = (ref_y(az + h, ti) - ref_y(az - h, ti)) / (2.0 * h) * w
                              + (ref_y(az, ti + h) - ref_y(az, ti - h)) / (2.0 * h) * r;
                    double ex = fabs(nx - rx), ey = fabs(ny - ry);
                    if (ex > worst_abs) worst_abs = ex;
                    if (ey > worst_abs) worst_abs = ey;
                    /* float32 evaluation error scales with magnitude */
                    if (ex / (fabs(nx) + 1.0) > worst_rel) worst_rel = ex / (fabs(nx) + 1.0);
                    if (ey / (fabs(ny) + 1.0) > worst_rel) worst_rel = ey / (fabs(ny) + 1.0);
                    n++;
                }
            }
        }
        printf("  compared %d points, worst abs %.2e deg/s, worst scaled %.2e\n",
               n, worst_abs, worst_rel);
        /* The float32 floor here is ~1.4e-5 scaled (a few ULP on rates reaching
         * 200 deg/s); in double the same formula agrees to 5e-6 deg/s. A real
         * derivation error -- wrong sign, missing factor -- lands at relative
         * 0.1 or worse, so this threshold still separates the two by four
         * orders of magnitude. */
        check(n > 0 && worst_rel < 1e-4, "analytic rate matches numeric derivative");
    }

    printf("a zero tilt rate reproduces the azimuth-only result exactly\n");
    {
        /* The tilt term is additive, so this is what keeps the extension
         * backward compatible for any caller that parks the tilt axis. */
        for (float az = 0; az < 360.0f; az += 21.0f) {
            float ax, ay, bx, by;
            plate_rates_from_spin(az, 14.0f, 90.0f, 0.0f, &ax, &ay);
            /* independent restatement of the old formula */
            const float t = tanf(14.0f * (float)DEG);
            const float cxx = cosf(atan2f(sinf(14.0f * (float)DEG) * cosf(az * (float)DEG),
                                          cosf(14.0f * (float)DEG)));
            const float cyy = cosf(atan2f(-sinf(14.0f * (float)DEG) * sinf(az * (float)DEG),
                                          cosf(14.0f * (float)DEG)));
            bx = -90.0f * t * sinf(az * (float)DEG) * cxx * cxx;
            by = -90.0f * t * cosf(az * (float)DEG) * cyy * cyy;
            check_near(ax, bx, 1e-4f, "tilt rate 0 -> x matches azimuth-only formula");
            check_near(ay, by, 1e-4f, "tilt rate 0 -> y matches azimuth-only formula");
        }
    }

    printf("tilting straight up the lean direction moves only that motor\n");
    {
        /* Azimuth 0 leans purely toward X, so opening the tilt there is a pure
         * toward-X motion and the toward-Y motor must stay still. */
        float rx, ry;
        plate_rates_from_spin(0.0f, 10.0f, 0.0f, 5.0f, &rx, &ry);
        check_near(ry, 0.0f, 1e-4f, "az 0, pure tilt -> toward-Y motor still");
        check_near(rx, 5.0f, 1e-3f, "az 0, pure tilt -> toward-X motor tracks tilt 1:1");
        /* Azimuth 90 leans purely toward Y; sign is the right-hand rule again. */
        plate_rates_from_spin(90.0f, 10.0f, 0.0f, 5.0f, &rx, &ry);
        check_near(rx,  0.0f, 1e-4f, "az 90, pure tilt -> toward-X motor still");
        check_near(ry, -5.0f, 1e-3f, "az 90, pure tilt -> toward-Y motor tracks -tilt");
    }

    printf("reversing the tilt ramp reverses both motor rates\n");
    {
        float fx, fy, bx, by;
        plate_rates_from_spin(30.0f, 12.0f, 0.0f,  7.0f, &fx, &fy);
        plate_rates_from_spin(30.0f, 12.0f, 0.0f, -7.0f, &bx, &by);
        check_near(bx, -fx, 1e-5f, "reversed tilt ramp negates x rate");
        check_near(by, -fy, 1e-5f, "reversed tilt ramp negates y rate");
    }

    printf("zero spin rate gives zero motor rates\n");
    {
        float rx = 5.0f, ry = 5.0f;
        check(plate_rates_from_spin(45.0f, 10.0f, 0.0f, 0.0f, &rx, &ry) == PLATE_OK, "rate 0 accepted");
        check_near(rx, 0.0f, 1e-6f, "rate 0 -> x rate 0");
        check_near(ry, 0.0f, 1e-6f, "rate 0 -> y rate 0");
    }

    printf("motor at its peak angle has zero rate; the other is at maximum\n");
    {
        /* azimuth 0 puts the toward-X motor at full lean, so its rate is 0 and
         * the toward-Y motor is sweeping fastest. Azimuth 90 swaps them. */
        float rx, ry;
        plate_rates_from_spin(0.0f, 10.0f, 90.0f, 0.0f, &rx, &ry);
        check_near(rx, 0.0f, 1e-3f, "az 0 -> toward-X motor at peak, rate 0");
        check(fabsf(ry) > 1.0f, "az 0 -> toward-Y motor sweeping");
        plate_rates_from_spin(90.0f, 10.0f, 90.0f, 0.0f, &rx, &ry);
        check_near(ry, 0.0f, 1e-3f, "az 90 -> toward-Y motor at peak, rate 0");
        check(fabsf(rx) > 1.0f, "az 90 -> toward-X motor sweeping");
    }

    printf("reversing the spin reverses both motor rates\n");
    {
        float fx, fy, bx, by;
        plate_rates_from_spin(30.0f, 12.0f,  90.0f, 0.0f, &fx, &fy);
        plate_rates_from_spin(30.0f, 12.0f, -90.0f, 0.0f, &bx, &by);
        check_near(bx, -fx, 1e-5f, "reversed spin negates x rate");
        check_near(by, -fy, 1e-5f, "reversed spin negates y rate");
    }

    printf("spin rates reject the same inputs the angles do\n");
    {
        float rx = 42.0f, ry = 42.0f;
        check(plate_rates_from_spin(0.0f, 31.0f, 90.0f, 0.0f, &rx, &ry) == PLATE_ERR_RANGE, "tilt 31 refused");
        check(plate_rates_from_spin(NAN, 10.0f, 90.0f, 0.0f, &rx, &ry) == PLATE_ERR_BAD_ARG, "NaN azimuth refused");
        check(plate_rates_from_spin(0.0f, 10.0f, NAN, 0.0f, &rx, &ry) == PLATE_ERR_BAD_ARG, "NaN azimuth rate refused");
        check(plate_rates_from_spin(0.0f, 10.0f, INFINITY, 0.0f, &rx, &ry) == PLATE_ERR_BAD_ARG, "Inf azimuth rate refused");
        check(plate_rates_from_spin(0.0f, 10.0f, 90.0f, NAN, &rx, &ry) == PLATE_ERR_BAD_ARG, "NaN tilt rate refused");
        check(plate_rates_from_spin(0.0f, 10.0f, 90.0f, INFINITY, &rx, &ry) == PLATE_ERR_BAD_ARG, "Inf tilt rate refused");
        check_near(rx, 42.0f, 0.0f, "refused call leaves x rate untouched");
        check_near(ry, 42.0f, 0.0f, "refused call leaves y rate untouched");
        check(plate_rates_from_spin(0.0f, 10.0f, 90.0f, 0.0f, NULL, &ry) == PLATE_ERR_BAD_ARG, "NULL out_x refused");
        check(plate_rates_from_spin(0.0f, 10.0f, 90.0f, 0.0f, &rx, NULL) == PLATE_ERR_BAD_ARG, "NULL out_y refused");
    }

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
