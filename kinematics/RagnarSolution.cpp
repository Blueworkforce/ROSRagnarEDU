#include "RagnarSolution.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"
#include "utils.h"

#include <fastmath.h>
#include "Vector3.h"

#define ragnar_joint1_base_x_checksum              CHECKSUM("ragnar_joint1_base_x")
#define ragnar_joint1_base_y_checksum              CHECKSUM("ragnar_joint1_base_y")
#define ragnar_joint1_base_pan_checksum            CHECKSUM("ragnar_joint1_base_pan")
#define ragnar_joint1_base_tilt_checksum           CHECKSUM("ragnar_joint1_base_tilt")
#define ragnar_joint1_primary_arm_checksum         CHECKSUM("ragnar_joint1_primary_arm")
#define ragnar_joint1_secondary_arm_checksum       CHECKSUM("ragnar_joint1_secondary_arm")
#define ragnar_joint1_bridge_distance_checksum     CHECKSUM("ragnar_joint1_bridge_distance")
#define ragnar_joint1_fixed_bridge_angle_checksum  CHECKSUM("ragnar_joint1_fixed_bridge_angle")
#define ragnar_joint1_hash_bridge_angle_checksum   CHECKSUM("ragnar_joint1_hash_bridge_angle")
#define ragnar_joint1_x_bridge_angle_checksum      CHECKSUM("ragnar_joint1_x_bridge_angle")

#define ragnar_joint2_base_x_checksum              CHECKSUM("ragnar_joint2_base_x")
#define ragnar_joint2_base_y_checksum              CHECKSUM("ragnar_joint2_base_y")
#define ragnar_joint2_base_pan_checksum            CHECKSUM("ragnar_joint2_base_pan")
#define ragnar_joint2_base_tilt_checksum           CHECKSUM("ragnar_joint2_base_tilt")
#define ragnar_joint2_primary_arm_checksum         CHECKSUM("ragnar_joint2_primary_arm")
#define ragnar_joint2_secondary_arm_checksum       CHECKSUM("ragnar_joint2_secondary_arm")
#define ragnar_joint2_bridge_distance_checksum     CHECKSUM("ragnar_joint2_bridge_distance")
#define ragnar_joint2_bridge_angle_checksum        CHECKSUM("ragnar_joint2_bridge_angle")
#define ragnar_joint2_fixed_bridge_angle_checksum  CHECKSUM("ragnar_joint2_fixed_bridge_angle")
#define ragnar_joint2_hash_bridge_angle_checksum   CHECKSUM("ragnar_joint2_hash_bridge_angle")
#define ragnar_joint2_x_bridge_angle_checksum      CHECKSUM("ragnar_joint2_x_bridge_angle")

#define ragnar_joint3_base_x_checksum              CHECKSUM("ragnar_joint3_base_x")
#define ragnar_joint3_base_y_checksum              CHECKSUM("ragnar_joint3_base_y")
#define ragnar_joint3_base_pan_checksum            CHECKSUM("ragnar_joint3_base_pan")
#define ragnar_joint3_base_tilt_checksum           CHECKSUM("ragnar_joint3_base_tilt")
#define ragnar_joint3_primary_arm_checksum         CHECKSUM("ragnar_joint3_primary_arm")
#define ragnar_joint3_secondary_arm_checksum       CHECKSUM("ragnar_joint3_secondary_arm")
#define ragnar_joint3_bridge_distance_checksum     CHECKSUM("ragnar_joint3_bridge_distance")
#define ragnar_joint3_bridge_angle_checksum        CHECKSUM("ragnar_joint3_bridge_angle")
#define ragnar_joint3_fixed_bridge_angle_checksum  CHECKSUM("ragnar_joint3_fixed_bridge_angle")
#define ragnar_joint3_hash_bridge_angle_checksum   CHECKSUM("ragnar_joint3_hash_bridge_angle")
#define ragnar_joint3_x_bridge_angle_checksum      CHECKSUM("ragnar_joint3_x_bridge_angle")

#define ragnar_joint4_base_x_checksum              CHECKSUM("ragnar_joint4_base_x")
#define ragnar_joint4_base_y_checksum              CHECKSUM("ragnar_joint4_base_y")
#define ragnar_joint4_base_pan_checksum            CHECKSUM("ragnar_joint4_base_pan")
#define ragnar_joint4_base_tilt_checksum           CHECKSUM("ragnar_joint4_base_tilt")
#define ragnar_joint4_primary_arm_checksum         CHECKSUM("ragnar_joint4_primary_arm")
#define ragnar_joint4_secondary_arm_checksum       CHECKSUM("ragnar_joint4_secondary_arm")
#define ragnar_joint4_bridge_distance_checksum     CHECKSUM("ragnar_joint4_bridge_distance")
#define ragnar_joint4_bridge_angle_checksum        CHECKSUM("ragnar_joint4_bridge_angle")
#define ragnar_joint4_fixed_bridge_angle_checksum  CHECKSUM("ragnar_joint4_fixed_bridge_angle")
#define ragnar_joint4_hash_bridge_angle_checksum   CHECKSUM("ragnar_joint4_hash_bridge_angle")
#define ragnar_joint4_x_bridge_angle_checksum      CHECKSUM("ragnar_joint4_x_bridge_angle")

#define ragnar_platform_hash_theta_min_checksum    CHECKSUM("ragnar_platform_hash_theta_min")
#define ragnar_platform_hash_theta_max_checksum    CHECKSUM("ragnar_platform_hash_theta_max")
#define ragnar_platform_x_theta_min_checksum       CHECKSUM("ragnar_platform_x_theta_min")
#define ragnar_platform_x_theta_max_checksum       CHECKSUM("ragnar_platform_x_theta_max")

#define ragnar_platform_solution_checksum          CHECKSUM("ragnar_platform_solution")
#define platform_fixed_checksum                    CHECKSUM("fixed")
#define platform_x_checksum                        CHECKSUM("x")
#define platform_hash_checksum                     CHECKSUM("hash")


#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * (float)(1e ## y)) / (float)(1e ## y))
#define PIOVER180   0.01745329251994329576923690768489F

RagnarSolution::RagnarSolution(Config* config)
{
	int platform_checksum = get_checksum(config->value(ragnar_platform_solution_checksum)->by_default("fixed")->as_string());

	if(platform_checksum == platform_fixed_checksum) {
		platform = FIXED;

		// lock rotation on fixed platform
		theta_min = 0.0F;
		theta_max = 0.0F;

		parameter[28] = config->value(ragnar_joint1_fixed_bridge_angle_checksum)->by_default(1.04719755F)->as_number();
		parameter[29] = config->value(ragnar_joint2_fixed_bridge_angle_checksum)->by_default(2.61799388F)->as_number();
		parameter[30] = config->value(ragnar_joint3_fixed_bridge_angle_checksum)->by_default(3.66519143F)->as_number();
		parameter[31] = config->value(ragnar_joint4_fixed_bridge_angle_checksum)->by_default(5.23598776F)->as_number();
	}
	else if (platform_checksum == platform_x_checksum) {
		platform = X_PLATFORM;

		theta_min = config->value(ragnar_platform_x_theta_min_checksum)->by_default(-30.0F)->as_number();
		theta_max = config->value(ragnar_platform_x_theta_max_checksum)->by_default(30.0F)->as_number();

		parameter[28] = config->value(ragnar_joint1_x_bridge_angle_checksum)->by_default(0.523598776F)->as_number();
		parameter[29] = config->value(ragnar_joint2_x_bridge_angle_checksum)->by_default(2.617993878F)->as_number();
		parameter[30] = config->value(ragnar_joint3_x_bridge_angle_checksum)->by_default(3.665191429F)->as_number();
		parameter[31] = config->value(ragnar_joint4_x_bridge_angle_checksum)->by_default(5.759586532F)->as_number();
	}
	else if (platform_checksum == platform_hash_checksum) {
		platform = HASH;

		theta_min = config->value(ragnar_platform_hash_theta_min_checksum)->by_default(-90.0F)->as_number();
		theta_max = config->value(ragnar_platform_hash_theta_max_checksum)->by_default(90.0F)->as_number();

		parameter[28] = config->value(ragnar_joint1_fixed_bridge_angle_checksum)->by_default(0.523598776F)->as_number();
		parameter[29] = config->value(ragnar_joint2_fixed_bridge_angle_checksum)->by_default(2.617993878F)->as_number();
		parameter[30] = config->value(ragnar_joint3_fixed_bridge_angle_checksum)->by_default(3.665191429F)->as_number();
		parameter[31] = config->value(ragnar_joint4_fixed_bridge_angle_checksum)->by_default(5.759586532F)->as_number();
	}

	// a : base_x
	parameter[0] = config->value(ragnar_joint1_base_x_checksum)->by_default(500.0f)->as_number();
	parameter[1] = config->value(ragnar_joint2_base_x_checksum)->by_default(-500.0f)->as_number();
	parameter[2] = config->value(ragnar_joint3_base_x_checksum)->by_default(-500.0f)->as_number();
	parameter[3] = config->value(ragnar_joint4_base_x_checksum)->by_default(500.0f)->as_number();
	
	// b : base_y
	parameter[4] = config->value(ragnar_joint1_base_y_checksum)->by_default(200.0f)->as_number();
	parameter[5] = config->value(ragnar_joint2_base_y_checksum)->by_default(200.0f)->as_number();
	parameter[6] = config->value(ragnar_joint3_base_y_checksum)->by_default(-200.0f)->as_number();
	parameter[7] = config->value(ragnar_joint4_base_y_checksum)->by_default(-200.0f)->as_number();
	
	// rho : base_pan
	parameter[8]  = config->value(ragnar_joint1_base_pan_checksum)->by_default(0.0f)->as_number();
	parameter[9]  = config->value(ragnar_joint2_base_pan_checksum)->by_default(0.0f)->as_number();
	parameter[10] = config->value(ragnar_joint3_base_pan_checksum)->by_default(0.0f)->as_number();
	parameter[11] = config->value(ragnar_joint4_base_pan_checksum)->by_default(0.0f)->as_number();

	// alpha : base_tilt
	parameter[12] = config->value(ragnar_joint1_base_tilt_checksum)->by_default(0.0f)->as_number();
	parameter[13] = config->value(ragnar_joint2_base_tilt_checksum)->by_default(0.0f)->as_number();
	parameter[14] = config->value(ragnar_joint3_base_tilt_checksum)->by_default(0.0f)->as_number();
	parameter[15] = config->value(ragnar_joint4_base_tilt_checksum)->by_default(0.0f)->as_number();	
			
    // l1 : primary_arm
	parameter[16] = config->value(ragnar_joint1_primary_arm_checksum)->by_default(0.0f)->as_number();
	parameter[17] = config->value(ragnar_joint2_primary_arm_checksum)->by_default(0.0f)->as_number();
	parameter[18] = config->value(ragnar_joint3_primary_arm_checksum)->by_default(0.0f)->as_number();
	parameter[19] = config->value(ragnar_joint4_primary_arm_checksum)->by_default(0.0f)->as_number();	

    // l2 : secondary_arm
	parameter[20] = config->value(ragnar_joint1_secondary_arm_checksum)->by_default(0.0f)->as_number();
	parameter[21] = config->value(ragnar_joint2_secondary_arm_checksum)->by_default(0.0f)->as_number();
	parameter[22] = config->value(ragnar_joint3_secondary_arm_checksum)->by_default(0.0f)->as_number();
	parameter[23] = config->value(ragnar_joint4_secondary_arm_checksum)->by_default(0.0f)->as_number();
	
	// c : bridge_distance
	parameter[24] = config->value(ragnar_joint1_bridge_distance_checksum)->by_default(0.0f)->as_number();
	parameter[25] = config->value(ragnar_joint2_bridge_distance_checksum)->by_default(0.0f)->as_number();
	parameter[26] = config->value(ragnar_joint3_bridge_distance_checksum)->by_default(0.0f)->as_number();
	parameter[27] = config->value(ragnar_joint4_bridge_distance_checksum)->by_default(0.0f)->as_number();
		
    init();
}

void RagnarSolution::init() {
	rad2deg = 180.0/acosf(-1.0);
	pi12 = acosf(0.0);

	// precalc cos
	// rho
	cp[8]  = cosf(parameter[8]);
	cp[9]  = cosf(parameter[9]);
	cp[10] = cosf(parameter[10]);
	cp[11] = cosf(parameter[11]);
	
	// alpha
	cp[12] = cosf(parameter[12]);
	cp[13] = cosf(parameter[13]);
	cp[14] = cosf(parameter[14]);
	cp[15] = cosf(parameter[15]);
	
	cp[16] = parameter[16];
	cp[17] = parameter[17];
	cp[18] = parameter[18];
	cp[19] = parameter[19];
	
	cp[20] = parameter[20];
	cp[21] = parameter[21];
	cp[22] = parameter[22];
	cp[23] = parameter[23];
	
	cp[24] = parameter[24];
	cp[25] = parameter[25];
	cp[26] = parameter[26];
	cp[27] = parameter[27];
	
	// beta
	cp[28] = cosf(parameter[28]);
	cp[29] = cosf(parameter[29]);
	cp[30] = cosf(parameter[30]);
	cp[31] = cosf(parameter[31]);
	
	// precalc sin
	// rho
	sp[8]  = sinf(parameter[8]);
	sp[9]  = sinf(parameter[9]);
	sp[10] = sinf(parameter[10]);
	sp[11] = sinf(parameter[11]);
	
	// alpha
	sp[12] = sinf(parameter[12]);
	sp[13] = sinf(parameter[13]);
	sp[14] = sinf(parameter[14]);
	sp[15] = sinf(parameter[15]);
	
	sp[16] = parameter[16];
	sp[17] = parameter[17];
	sp[18] = parameter[18];
	sp[19] = parameter[19];
	
	sp[20] = parameter[20];
	sp[21] = parameter[21];
	sp[22] = parameter[22];
	sp[23] = parameter[23];
	
	sp[24] = parameter[24];
	sp[25] = parameter[25];
	sp[26] = parameter[26];
	sp[27] = parameter[27];	

	// beta
	sp[28] = sinf(parameter[28]);
	sp[29] = sinf(parameter[29]);
	sp[30] = sinf(parameter[30]);
	sp[31] = sinf(parameter[31]);
	
	if(parameter[16] == 0.0f || parameter[17] == 0.0f || parameter[18] == 0.0f || parameter[19] == 0.0f ||
	   parameter[20] == 0.0f || parameter[21] == 0.0f || parameter[22] == 0.0f || parameter[23] == 0.0f)
		this->isValid = false;
	else
		this->isValid = true;
}

void RagnarSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] )
{
	if(!isValid)
		return;
			
	float Q[9];
	int k;
	static const signed char iv0[3] = { 0, 0, 1 };

	float SOL[8];
	int i;
	float leg[8],sLeg[8],cLeg[8];

	float dv0[3];
	float b_Q[3];
	float b_leg[3];
	float CA[3];
	int i0;
	float C[3];
	float I;
	float J;
	float K;

	// limit theta to platform restrictions
	if(cartesian_mm[3] > theta_max)
		cartesian_mm[3] = theta_max;

	if(cartesian_mm[3] < theta_min)
		cartesian_mm[3] = theta_min;

	float theta = cartesian_mm[3] * PIOVER180;

	Q[0] = cosf(theta);
	Q[3] = -sinf(theta);
	Q[6] = 0.0;
	Q[1] = sinf(theta);
	Q[4] = cosf(theta);
	Q[7] = 0.0;
	for (k = 0; k < 3; k++) {
		Q[2 + 3 * k] = iv0[k];
	}

	for (i = 0; i < 4; i++) {
		for (k = 0; k < 8; k++) {
			leg[k] = parameter[i + (k << 2)];
			sLeg[k] = sp[i + (k << 2)];
			cLeg[k] = cp[i + (k << 2)];
		}

		dv0[0] = cLeg[7];
		dv0[1] = sLeg[7];
		dv0[2] = 0.0;

		b_leg[0] = leg[0];
		b_leg[1] = leg[1];
		b_leg[2] = 0.0;
		for (k = 0; k < 3; k++) {
			b_Q[k] = 0.0;
			for (i0 = 0; i0 < 3; i0++) {
				b_Q[k] += Q[k + 3 * i0] * dv0[i0];
			}

			C[k] = b_Q[k] * leg[6] + cartesian_mm[k];
			CA[k] = C[k] - b_leg[k];
		}

		I = 2.0 * leg[4] * ((C[0] - leg[0]) * sLeg[2] - (C[1] - leg[1]) * cLeg[2]);
		J = -2.0 * leg[4] * (((C[0] - leg[0]) * cLeg[2] * cLeg[3] + (C[1] - leg[1]) * sLeg[2] * cLeg[3]) - C[2] * sLeg[3]);

		K = 0.0;
		for (k = 0; k < 3; k++) {
			K += CA[k] * CA[k];
		}

		K = (K + leg[4] * leg[4]) - leg[5] * leg[5];

		SOL[i << 1] = 2.0 * atan((-I + sqrt((I * I + J * J) - K * K)) / (K - J));
		SOL[1 + (i << 1)] = 2.0 * atan((-I - sqrt((I * I + J * J) - K * K)) / (K - J));
	}

	actuator_mm[ALPHA_STEPPER] = (SOL[1]-pi12)*rad2deg;
	actuator_mm[BETA_STEPPER]  = (SOL[2]-pi12)*rad2deg;
	actuator_mm[GAMMA_STEPPER] = (pi12+SOL[5])*rad2deg;
	actuator_mm[DELTA_STEPPER] = (pi12+SOL[6])*rad2deg;	
}

void RagnarSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] )
{
    float joints[4] = { (actuator_mm[0] + 90) * PIOVER180, (actuator_mm[1] + 90) * PIOVER180, (actuator_mm[2] - 90) * PIOVER180, (actuator_mm[3] - 90) * PIOVER180 };
    float u1, v1, w1;
    float u2, v2, w2;
    float u3, v3, w3;
    float s1x, s1y, s1z;
    float s2x, s2y, s2z;
    float s11, s22;
    float D[4];
    float g[2];
    int r1, r2;
    float xy[2];
    float sxx, sxy, sxz;

    u1 = (-parameter[0] + parameter[24] * cp[28]) + parameter[16] * (sp[8] * sinf(joints[0]) - cp[8] * cp[12] * cosf(joints[0]));
    v1 = -((parameter[4] - parameter[24] * sp[28]) + parameter[16] * (cp[8] * sinf(joints[0]) + cp[12] * sp[8] * cosf(joints[0])));
    w1 = parameter[16] * sp[12] * cosf(joints[0]);

    u2 = (-parameter[1] + parameter[25] * cp[29]) + parameter[17] * (sp[9] * sinf(joints[1]) - cp[9] * cp[13] * cosf(joints[1]));
    v2 = -((parameter[5] - parameter[25] * sp[29]) + parameter[17] * (cp[9] * sinf(joints[1]) + cp[13] * sp[9] * cosf(joints[1])));
    w2 = parameter[17] * sp[13] * cosf(joints[1]);

    u3 = (-parameter[2] + parameter[26] * cp[30]) + parameter[18] * (sp[10] * sinf(joints[2]) - cp[10] * cp[14] * cosf(joints[2]));
    v3 = -((parameter[6] - parameter[26] * sp[30]) + parameter[18] * (cp[10] * sinf(joints[2]) + cp[14] * sp[10] * cosf(joints[2])));
    w3 = parameter[18] * sp[14] * cosf(joints[2]);

    s1x = 2.0 * u1 - 2.0 * u2;
    s1y = 2.0 * v1 - 2.0 * v2;
    s1z = 2.0 * w1 - 2.0 * w2;
    s11 = ((((powf(u1, 2.0) - powf(u2, 2.0)) + powf(v1, 2.0)) - powf(v2, 2.0)) + powf(w1, 2.0)) - powf(w2, 2.0);
    s2x = 2.0 * u1 - 2.0 * u3;
    s2y = 2.0 * v1 - 2.0 * v3;
    s2z = 2.0 * w1 - 2.0 * w3;
    s22 = ((((powf(u1, 2.0) - powf(u3, 2.0)) + powf(v1, 2.0)) - powf(v3, 2.0)) + powf(w1, 2.0)) - powf(w3, 2.0);

    if ((joints[0] == 1.5707963267948966) && (joints[1] == 1.5707963267948966) && (joints[2] == -1.5707963267948966)) {
        D[0] = s1x; D[2] = s1y; D[1] = s2x; D[3] = s2y;
        g[0] = -s11; g[1] = -s22;

        if (fabs(D[1]) > fabs(D[0])) {
            r1 = 1;
            r2 = 0;
        }
        else
        {
            r1 = 0;
            r2 = 1;
        }

        v2 = D[r2] / D[r1];
        u2 = D[2 + r2] - v2 * D[2 + r1];
        xy[1] = (g[r2] - g[r1] * v2) / u2;
        xy[0] = (g[r1] - xy[1] * D[2 + r1]) / D[r1];

        for (r1 = 0; r1 < 2; r1++) {
            cartesian_mm[r1] = xy[r1];
        }

        cartesian_mm[2] = -sqrtf((powf(parameter[20], 2.0) - powf(xy[0] + u1, 2.0)) - powf(xy[1] + v1, 2.0));
    }
    else
    {
        sxx = (powf(s1x * s2y - s2x * s1y, 2.0) / powf(s1y * s2z - s2y * s1z, 2.0) + powf(s1x * s2z - s2x * s1z, 2.0) / powf(s1y * s2z - s2y * s1z, 2.0)) + 1.0;
        sxy = (((2.0 * u1 - 2.0 * v1 * (s1x * s2z - s2x * s1z) / (s1y * s2z - s2y * s1z)) + 2.0 * w1 * (s1x * s2y - s2x * s1y) / (s1y * s2z - s2y * s1z))
                + 2.0 * (s11 * s2y - s22 * s1y) * (s1x * s2y - s2x * s1y) / powf(s1y * s2z - s2y * s1z, 2.0)) + 2.0 * (s11 * s2z - s22 * s1z) * (s1x * s2z - s2x * s1z) / powf(s1y * s2z - s2y * s1z, 2.0);
        sxz = (powf(u1, 2.0) - powf(parameter[20], 2.0) + powf(v1, 2.0) + powf(w1, 2.0)) + powf(s11 * s2y - s22 * s1y, 2.0) / powf(s1y * s2z - s2y * s1z, 2.0)
                + powf(s11 * s2z - s22 * s1z, 2.0) / powf(s1y * s2z - s2y * s1z, 2.0) - 2.0 * v1 * (s11 * s2z - s22 * s1z) / (s1y * s2z - s2y * s1z) + 2.0 * w1 * (s11 * s2y - s22 * s1y) / (s1y * s2z - s2y * s1z);

        cartesian_mm[0] = (-sxy + sqrtf( powf(sxy, 2.0) - 4.0 * sxx * sxz))/(2.0 * sxx);
        cartesian_mm[1] = -(((s11 * s2z - s22 * s1z) + s1x * s2z * cartesian_mm[0]) - s2x * s1z * cartesian_mm[0]) / (s1y * s2z - s2y * s1z);
        cartesian_mm[2] = (((s11 * s2y - s22 * s1y) + s1x * s2y * cartesian_mm[0]) - s2x * s1y * cartesian_mm[0]) / (s1y * s2z - s2y * s1z);
        if (cartesian_mm[2] > 0)  {
            cartesian_mm[0] = (-sxy - sqrtf(powf(sxy, 2.0) - 4.0 * sxx * sxz))/(2.0 * sxx);
            cartesian_mm[1] = -(((s11 * s2z - s22 * s1z) + s1x * s2z * cartesian_mm[0]) - s2x * s1z * cartesian_mm[0]) / (s1y * s2z - s2y * s1z);
            cartesian_mm[2] = (((s11 * s2y - s22 * s1y) + s1x * s2y * cartesian_mm[0]) - s2x * s1y * cartesian_mm[0]) / (s1y * s2z - s2y * s1z);
        }
    }
}

bool RagnarSolution::set_optional(const arm_options_t& options) {

    for(auto &i : options) {
        switch(i.first) {
            case 'a': this->parameter[16] = i.second; break;
            case 'b': this->parameter[17] = i.second; break;
            case 'c': this->parameter[18] = i.second; break;
            case 'd': this->parameter[19] = i.second; break;
            case 'A': this->parameter[20] = i.second; break;
            case 'B': this->parameter[21] = i.second; break;
            case 'C': this->parameter[22] = i.second; break;
            case 'D': this->parameter[23] = i.second; break;
        }
    }

    init();
    return true;
}

bool RagnarSolution::get_optional(arm_options_t& options) {
    options['a']= this->parameter[16]; // joint 1 primary arm
    options['b']= this->parameter[17];
    options['c']= this->parameter[18];
    options['d']= this->parameter[19];
    options['A']= this->parameter[20]; // joint 1 secondary arm
    options['B']= this->parameter[21];
    options['C']= this->parameter[22];
    options['D']= this->parameter[23];

    return true;
};
