#include "stdafx.h"
#include "code.h"

int winWidth = 640;		// window width
int winHeight = 480;	// window height

V3 ViewPoint;		// view point
V3 ImageLL;		// coordinates at lower left corner of image
V3 ImageLR;		// coordinates at lower right corner of image
V3 ImageUL;		// coordinates at upper left corner of image
V3 ImageUR;		// coordinates at upper right corner of image

int MaxTraceDepth = 5;			// depth of recursive ray-tracing

// scene objects
vector<CLightSource *> vLightSource;		// array of light sources
vector<CPrimitive *> vObjects;				// array of objects

void RayTracing(V3 * colorMap)
{	
	//add your code here
	int i,j;
	V3 cur_point,rayStart,rayDir,color(0.0,0.0,0.0);
	for(i=0 ; i<winWidth ; i++)
	{
		for(j=0 ; j<winHeight ; j++)
		{
			cur_point = ImageLL + i*(ImageLR-ImageLL)/(winWidth-1)
								+ j*(ImageUL-ImageLL)/(winHeight-1);
			rayStart = ViewPoint;
			rayDir = cur_point - rayStart;
			rayDir.normalize();
			Trace(rayStart,rayDir,0,color);
			colorMap[j*winWidth+i] = color;
		}
	}
}


void Trace(V3& rayStart, V3& rayDir, int depth, V3& color)
{
	//add your code here
	CPrimitive* objHit;
	V3 intersection, normal;
	rayDir.normalize();
	if(Intersect(rayStart,rayDir,objHit,intersection,normal))
	{
		//normal.normalize();
		Shade(objHit, rayStart, rayDir,intersection,normal,depth,color);
	}else{
		color = V3(0,0,0);
	}
}


// compute color at a given point
void Shade(CPrimitive *obj, V3& rayStart, V3& rayDir, V3& intersection, V3& normal, int depth, V3& color)
{
	//add your code here
	int i;
	CPrimitive* tmp_obj;
	V3 cur_LightPos, Light_intersection, Light_normal;
	V3 Oa, Od, Os, Ipi, Inside_Square, Li , Ir, It, R, V;
	V3 Second_term(0,0,0), Third_term(0,0,0);
	float Kt, n, Ks;
	bool intersect_flag;
	/*
	Formula to calculate color at the intersection point.
	I = Oa + SUM{ Si*Ipi*[ (1 – kt )*Od*(N * Li) + ks*Os*(R * V)^n ]} + ks*Ir + kt*It
	SUM term is taken over all light sources.
	*/
	///////////////get ambient, diffuse, specular color////////////////
	obj->GetAmbient(intersection, Oa);
	obj->GetDiffuse(intersection, Od);
	obj->GetSpecular(intersection, Os);
	///////////////get ambient, diffuse, specular color////////////////

	Ks = obj->m_Reflectance;
	Kt = 1 - obj->m_Opacity;
	//normal.normalize();
	for(i=0 ; i<vLightSource.size() ; i++)
	{
		cur_LightPos = vLightSource[i]->position;
		Li = cur_LightPos - intersection;	Li.normalize();
		if(normal.dot(Li) > 0)
		{
			intersect_flag = Intersect(intersection,Li,tmp_obj,Light_intersection, Light_normal);
			if(intersect_flag == false || 
			   (intersect_flag == true &&
			   intersection.distance(Light_intersection) > intersection.distance(cur_LightPos)
			   )
			  )
			{
				//////////////////////////////Second term: Si*Ipi[...]///////////////////////////////
				R = 2 * normal.dot(Li) * normal - Li; R.normalize();
				Ipi = vLightSource[i]->color;
				V = rayStart - intersection; V.normalize();
				n = obj -> m_Shininess;
				Inside_Square = (1-Kt)*Od*(normal.dot(Li)) + Ks*Os*pow(R.dot(V)*1.0,n*1.0);
				Second_term += V3(Ipi[0]*Inside_Square[0],
								  Ipi[1]*Inside_Square[1],
								  Ipi[2]*Inside_Square[2]);
				//////////////////////////////Second term: Si*Ipi[...]///////////////////////////////
			}
		}
	}
	//////////////////////////////Third term: Ks*Ir///////////////////////////////
	if(depth < MaxTraceDepth)
	{
		if(Ks > 0)
		{
			rayDir.normalize();
			V3 Reflected_R(2*normal*normal.dot(-rayDir)+rayDir);
			Trace(intersection, Reflected_R, depth+1, Ir);
			Third_term += (Ks*Ir);
		}
	}
	//////////////////////////////Third term: Ks*Ir///////////////////////////////
	color = (Oa + Second_term + Third_term); 
	color = V3(color[0] > 1.0?1.0 : color[0],
			   color[1] > 1.0?1.0 : color[1],
			   color[2] > 1.0?1.0 : color[2]);//color clamping to one.
}



bool IntersectQuadratic(V3 rayStart,V3 rayDir, float * coeffMatrix, float& t, V3& intersection)
{
	//add your code here
	float S[] = {rayStart[0],rayStart[1],rayStart[2],1};
	float D[] = {rayDir[0],rayDir[1],rayDir[2],0};
	float a_[4],a,b_[4],b,c_[4],c,delta,t0,t1;
	//////////////////////////////Calculate a,b,c///////////////////////////////
	//a = D^T*A*D | b = 2*(S^T*A*D) | c = (S^T*A*S)
	VectorMultMatrix(D,coeffMatrix,a_);
	a = VectorMultVector(a_,D);

	VectorMultMatrix(S,coeffMatrix,b_);
	b = 2 * VectorMultVector(b_,D);

	VectorMultMatrix(S,coeffMatrix,c_);
	c = VectorMultVector(c_,S);
	//////////////////////////////Calculate a,b,c///////////////////////////////

	//////////////////////////////Find t////////////////////////////////////////
	delta = b*b - 4*a*c;

	if(delta < 0 || (delta == 0 && -b/(2*a)<=0)) return false;
	else
	{
		t0 = (-b+sqrt(delta))/(2.0*a);
		t1 = (-b-sqrt(delta))/(2.0*a);
		if(t0 <= 0 && t1 <= 0) return false;
		else if(t0 > 0 && (t1 >= t0 || t1 <= 0) ) t = t0;
		else if(t1 > 0 && (t0 > t1 || t0 <= 0) )  t = t1;
	}
	//////////////////////////////Find t////////////////////////////////////////
	intersection = V3(S[0]+D[0]*t , S[1]+D[1]*t , S[2]+D[2]*t);
	return true;
}

bool  IntersectTriangle(V3 rayStart,V3 rayDir, V3 v0, V3 v1,V3 v2, float& t,V3& intersection)
{	
	//add your code here
	rayDir.normalize();
	int i;
	V3 vec1_ = v1 - v0, vec2_ = v2 - v0, N, cross01, cross20, cross12;
	float S[] = {rayStart[0],rayStart[1],rayStart[2],1};
	float D[] = {rayDir[0],rayDir[1],rayDir[2],0};
	float d, C[4], vec1[] = {vec1_[0], vec1_[1], vec1_[2], 1};
	float vec2[] = {vec2_[0], vec2_[1], vec2_[2], 1};

	//////////////////////find plane equation////////////////////
	//Calculate N//
	N = vec1_.cross(vec2_); 
	//Calculate d//
	d = N[0] * v0[0] + N[1] * v0[1] + N[2] * v0[2];

	//////////////////////find plane equation////////////////////

	//////////////////////find t/////////////////////////////////
	if(N.dot(V3(D[0],D[1],D[2])) != 0)
		t = (d-N[0]*S[0]-N[1]*S[1]-N[2]*S[2])/(N[0]*D[0]+N[1]*D[1]+N[2]*D[2]);
	//////////////////////find t/////////////////////////////////

	for(i=0; i<4; i++)	C[i] = S[i] + D[i]*t;
	cross01 = V3(v0[0]-C[0], v0[1]-C[1], v0[2]-C[2]).cross(V3(v1[0]-C[0], v1[1]-C[1], v1[2]-C[2]));
	cross20 = V3(v2[0]-C[0], v2[1]-C[1], v2[2]-C[2]).cross(V3(v0[0]-C[0], v0[1]-C[1], v0[2]-C[2]));
	cross12 = V3(v1[0]-C[0], v1[1]-C[1], v1[2]-C[2]).cross(V3(v2[0]-C[0], v2[1]-C[1], v2[2]-C[2]));
	if(cross01.dot(cross20) < 0 || cross01.dot(cross12) < 0)	return false;
	intersection = V3(C[0]/C[3],C[1]/C[3],C[2]/C[3]);
	return true;
}


void MatrixMultVector(float *m,float *v,float *rv)//rv=m*v
{
	rv[0]=m[0]*v[0]+m[4]*v[1]+m[8]*v[2]+m[12]*v[3];
	rv[1]=m[1]*v[0]+m[5]*v[1]+m[9]*v[2]+m[13]*v[3];
	rv[2]=m[2]*v[0]+m[6]*v[1]+m[10]*v[2]+m[14]*v[3];
	rv[3]=m[3]*v[0]+m[7]*v[1]+m[11]*v[2]+m[15]*v[3];
}

void VectorMultMatrix(float *v,float *m,float *lv)//lv=v^Tm
{
	lv[0]=m[0]*v[0]+m[1]*v[1]+m[2]*v[2]+m[3]*v[3];
	lv[1]=m[4]*v[0]+m[5]*v[1]+m[6]*v[2]+m[7]*v[3];
	lv[2]=m[8]*v[0]+m[9]*v[1]+m[10]*v[2]+m[11]*v[3];
	lv[3]=m[12]*v[0]+m[13]*v[1]+m[14]*v[2]+m[15]*v[3];
}

float VectorMultVector(float *v1,float *v2)//v3=v1^Tv2
{
	return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]+v1[3]*v2[3];
}
