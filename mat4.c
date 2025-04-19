#include <math.h>
#include <stdint.h>
#include "mat4.h"

void mat4_cpy(float dst[restrict 16], const float src[16])
{
	for (uint_fast8_t i = 0; i < 16; i++)
		dst[i] = src[i];
}

void mat4_identity(float d[16])
{
	d[0] = 1.0f;
	d[1] = 0.0f;
	d[2] = 0.0f;
	d[3] = 0.0f;

	d[4] = 0.0f;
	d[5] = 1.0f;
	d[6] = 0.0f;
	d[7] = 0.0f;

	d[8] = 0.0f;
	d[9] = 0.0f;
	d[10] = 1.0f;
	d[11] = 0.0f;

	d[12] = 0.0f;
	d[13] = 0.0f;
	d[14] = 0.0f;
	d[15] = 1.0f;
}

void mat4_transpose(float dst[16], const float src[16])
{
	float s0 = src[0];
	float s1 = src[1];
	float s2 = src[2];
	float s3 = src[3];
	float s4 = src[4];
	float s5 = src[5];
	float s6 = src[6];
	float s7 = src[7];
	float s8 = src[8];
	float s9 = src[9];
	float s10 = src[10];
	float s11 = src[11];
	float s12 = src[12];
	float s13 = src[13];
	float s14 = src[14];
	float s15 = src[15];

	dst[0] = s0;
	dst[1] = s4;
	dst[2] = s8;
	dst[3] = s12;

	dst[4] = s1;
	dst[5] = s5;
	dst[6] = s9;
	dst[7] = s13;

	dst[8] = s2;
	dst[9] = s6;
	dst[10] = s10;
	dst[11] = s14;

	dst[12] = s3;
	dst[13] = s7;
	dst[14] = s11;
	dst[15] = s15;
}

void mat4_trans(float d[16], const struct vec3 v)
{
	d[0] = 1.0f;
	d[1] = 0.0f;
	d[2] = 0.0f;
	d[3] = v.x;

	d[4] = 0.0f;
	d[5] = 1.0f;
	d[6] = 0.0f;
	d[7] = v.y;

	d[8] = 0.0f;
	d[9] = 0.0f;
	d[10] = 1.0f;
	d[11] = v.z;

	d[12] = 0.0f;
	d[13] = 0.0f;
	d[14] = 0.0f;
	d[15] = 1.0f;
}

void mat4_rotx(float d[16], float rad)
{
	float c = cosf(rad);
	float s = sinf(rad);

	d[0] = 1.0f;
	d[1] = 0.0f;
	d[2] = 0.0f;
	d[3] = 0.0f;

	d[4] = 0.0f;
	d[5] = c;
	d[6] = -s;
	d[7] = 0.0f;

	d[8] = 0.0f;
	d[9] = s;
	d[10] = c;
	d[11] = 0.0f;

	d[12] = 0.0f;
	d[13] = 0.0f;
	d[14] = 0.0f;
	d[15] = 1.0f;
}

void mat4_roty(float d[16], float rad)
{
	float c = cosf(rad);
	float s = sinf(rad);

	d[0] = c;
	d[1] = 0.0f;
	d[2] = s;
	d[3] = 0.0f;

	d[4] = 0.0f;
	d[5] = 1.0f;
	d[6] = 0.0f;
	d[7] = 0.0f;

	d[8] = -s;
	d[9] = 0.0f;
	d[10] = c;
	d[11] = 0.0f;

	d[12] = 0.0f;
	d[13] = 0.0f;
	d[14] = 0.0f;
	d[15] = 1.0f;
}

void mat4_rotz(float d[16], float rad)
{
	float c = cosf(rad);
	float s = sinf(rad);

	d[0] = c;
	d[1] = -s;
	d[2] = 0.0f;
	d[3] = 0.0f;

	d[4] = s;
	d[5] = c;
	d[6] = 0.0f;
	d[7] = 0.0f;

	d[8] = 0.0f;
	d[9] = 0.0f;
	d[10] = 1.0f;
	d[11] = 0.0f;

	d[12] = 0.0f;
	d[13] = 0.0f;
	d[14] = 0.0f;
	d[15] = 1.0f;
}

void mat4_scale(float d[16], struct vec3 s)
{
	d[0] = s.x;
	d[1] = 0.0f;
	d[2] = 0.0f;
	d[3] = 0.0f;

	d[4] = 0.0f;
	d[5] = s.y;
	d[6] = 0.0f;
	d[7] = 0.0f;

	d[8] = 0.0f;
	d[9] = 0.0f;
	d[10] = s.z;
	d[11] = 0.0f;

	d[12] = 0.0f;
	d[13] = 0.0f;
	d[14] = 0.0f;
	d[15] = 1.0f;
}

void mat4_scaleu(float d[16], float s)
{
	d[0] = s;
	d[1] = 0.0f;
	d[2] = 0.0f;
	d[3] = 0.0f;

	d[4] = 0.0f;
	d[5] = s;
	d[6] = 0.0f;
	d[7] = 0.0f;

	d[8] = 0.0f;
	d[9] = 0.0f;
	d[10] = s;
	d[11] = 0.0f;

	d[12] = 0.0f;
	d[13] = 0.0f;
	d[14] = 0.0f;
	d[15] = 1.0f;
}

void mat4_mul(float dst[16], const float a[16], const float b[16])
{
	float a0 = a[0];
	float a1 = a[1];
	float a2 = a[2];
	float a3 = a[3];
	float a4 = a[4];
	float a5 = a[5];
	float a6 = a[6];
	float a7 = a[7];
	float a8 = a[8];
	float a9 = a[9];
	float a10 = a[10];
	float a11 = a[11];
	float a12 = a[12];
	float a13 = a[13];
	float a14 = a[14];
	float a15 = a[15];

	float b0 = b[0];
	float b1 = b[1];
	float b2 = b[2];
	float b3 = b[3];
	float b4 = b[4];
	float b5 = b[5];
	float b6 = b[6];
	float b7 = b[7];
	float b8 = b[8];
	float b9 = b[9];
	float b10 = b[10];
	float b11 = b[11];
	float b12 = b[12];
	float b13 = b[13];
	float b14 = b[14];
	float b15 = b[15];

	dst[0] = a0 * b0 + a1 * b4 + a2 * b8 + a3 * b12;
	dst[1] = a0 * b1 + a1 * b5 + a2 * b9 + a3 * b13;
	dst[2] = a0 * b2 + a1 * b6 + a2 * b10 + a3 * b14;
	dst[3] = a0 * b3 + a1 * b7 + a2 * b11 + a3 * b15;

	dst[4] = a4 * b0 + a5 * b4 + a6 * b8 + a7 * b12;
	dst[5] = a4 * b1 + a5 * b5 + a6 * b9 + a7 * b13;
	dst[6] = a4 * b2 + a5 * b6 + a6 * b10 + a7 * b14;
	dst[7] = a4 * b3 + a5 * b7 + a6 * b11 + a7 * b15;

	dst[8] = a8 * b0 + a9 * b4 + a10 * b8 + a11 * b12;
	dst[9] = a8 * b1 + a9 * b5 + a10 * b9 + a11 * b13;
	dst[10] = a8 * b2 + a9 * b6 + a10 * b10 + a11 * b14;
	dst[11] = a8 * b3 + a9 * b7 + a10 * b11 + a11 * b15;

	dst[12] = a12 * b0 + a13 * b4 + a14 * b8 + a15 * b12;
	dst[13] = a12 * b1 + a13 * b5 + a14 * b9 + a15 * b13;
	dst[14] = a12 * b2 + a13 * b6 + a14 * b10 + a15 * b14;
	dst[15] = a12 * b3 + a13 * b7 + a14 * b11 + a15 * b15;
}

struct vec3 mat4_mulpos(const float m[16], struct vec3 v)
{
	struct vec3 r = {
	  m[0] * v.x + m[1] * v.y + m[2] * v.z + m[3],
	  m[4] * v.x + m[5] * v.y + m[6] * v.z + m[7],
	  m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]};

	float w = m[12] * v.x + m[13] * v.y + m[14] * v.z + m[15];

	return w == 1 ? r : vec3_scale(r, 1.0f / w);
}

struct vec3 mat4_muldir(const float m[16], struct vec3 v)
{
	return (struct vec3){
	  m[0] * v.x + m[1] * v.y + m[2] * v.z,
	  m[4] * v.x + m[5] * v.y + m[6] * v.z,
	  m[8] * v.x + m[9] * v.y + m[10] * v.z};
}

// Taken from Mesa 3D
// https://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
int mat4_inv(float dst[16], const float m[16])
{
	float inv[16];

	inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
	         m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];

	inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] -
	          m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];

	inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
	         m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];

	inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] -
	           m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

	inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] -
	          m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];

	inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
	         m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];

	inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
	          m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];

	inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
	          m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

	inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
	         m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];

	inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
	          m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];

	inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
	          m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];

	inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
	           m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
	          m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
	         m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
	           m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
	          m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

	float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (det == 0)
		return 1;

	det = 1.0 / det;

	for (uint_fast8_t i = 0; i < 16; i++)
		dst[i] = inv[i] * det;

	return 0;
}

struct vec3 mat4_gettrans(const float m[16])
{
	return (struct vec3){m[3], m[7], m[11]};
}

void mat4_fromquat(float dst[16], float x, float y, float z, float w)
{
	float xx = x * x;
	float xy = x * y;
	float xz = x * z;
	float xw = x * w;
	float yy = y * y;
	float yz = y * z;
	float yw = y * w;
	float zz = z * z;
	float zw = z * w;

	dst[0] = 1.0f - 2.0f * (yy + zz);
	dst[1] = 2.0f * (xy - zw);
	dst[2] = 2.0f * (xz + yw);

	dst[4] = 2.0f * (xy + zw);
	dst[5] = 1.0f - 2.0f * (xx + zz);
	dst[6] = 2.0f * (yz - xw);

	dst[8] = 2.0f * (xz - yw);
	dst[9] = 2.0f * (yz + xw);
	dst[10] = 1.0f - 2.0f * (xx + yy);

	dst[3] = dst[7] = dst[11] = dst[12] = dst[13] = dst[14] = 0.0f;
	dst[15] = 1.0f;
}

void mat4_from3x4(float dst[16], const float src[12])
{
	for (uint_fast8_t i = 0; i < 12; i++)
		dst[i] = src[i];
	dst[12] = dst[13] = dst[14] = 0.0f;
	dst[15] = 1.0f;
}
