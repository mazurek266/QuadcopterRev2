#ifndef INC_DATASTRUCTURES_H_
#define INC_DATASTRUCTURES_H_

typedef struct
{
	float x;
	float y;
	float z;
}Vector3AxisF;
typedef struct
{
	float x;
	float y;
	float z;
}Vector2AxisF;
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Vector3AxisI;
typedef struct
{
	float P;
	float I;
	float D;
	float lastPosition;
	float integratedError;
}PIDdata;
struct Data
{
	float accelX;
	float accelY;
	float accelZ;
	float rotX;
	float rotY;
	float rotZ;
	float temp;
};

#endif
