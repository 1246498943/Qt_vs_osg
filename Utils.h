#pragma once
typedef struct AscDataD3Struct
{
	float u;
	float v;

	float x;
	float y;
	float z;

	unsigned char R;
	unsigned char G;
	unsigned char B;

	float normalX;
	float normalY;
	float normalZ;

	int ID;

}AscDataD3Struct;


enum CameraView
{
	LEFT,
	RIGHT,
	TOP,
	BOTTOM,
	FRONT,
	BACK
};
