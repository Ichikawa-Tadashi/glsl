#version 130

// simple.frag

#define PI 3.14159265359
#define MARCHING_DELTA 0.001
int REFLECT_NUM=10*0;
float REFLECT_RATE=0.3;
float SHADOW_RATE=0.5;
float SPECULAR_RATE=0.7;
// int REFRACTION_NUM=10;
float REFRACTION_RATE=0.9;//屈折での減衰
vec3 REFRACTION_MATERIAL=vec3(0.9,0.9,0.9);
// float REFRACTION_INDEX=1.5;//屈折率 水は1.33 ガラスは1.4~2.0
float REFRACTION_INDEX=1.5;//屈折率 水は1.33 ガラスは1.4~2.0

vec3 NOREFLECT_MATERIAL=vec3(0.9,0.8,0.7);

int STAGE=0;

varying vec4 position;
varying vec3 normal;

uniform vec2 resolution;
uniform vec2 mouse_pos;
uniform vec2 cursor_pos;
uniform vec2 camera_ang;
uniform vec3 CameraPos;
uniform float time;

//3. Rayの定義 (構造体)
struct Ray{
    vec3 pos; //Rayの現在の座標
    vec3 dir; //Rayの進行方向
};

float NORM_DELTA=0.001;
vec3 DX=vec3(NORM_DELTA,0.0,0.0);
vec3 DY=vec3(0.0,NORM_DELTA,0.0);
vec3 DZ=vec3(0.0,0.0,NORM_DELTA);


//12. 回転行列(X軸)
mat3 x_axis_rot(float angle){
    float c = cos(angle);
    float s = sin(angle);
    return mat3(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c); 
}

//12. 回転行列(Y軸)
mat3 y_axis_rot(float angle){
    float c = cos(angle);
    float s = sin(angle);
    return mat3(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0,  c);
}

//12. 回転行列(Z軸)
mat3 z_axis_rot(float angle){
    float c = cos(angle);
    float s = sin(angle);
    return mat3(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1);
}


float sdSphere( vec3 p, float s){
  return length(p)-s;
  //return length(p)-1.5*(sin(time*30  *PI/180)+1)/2;
}

float sdTorus( vec3 p , vec2 t){
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

// 16. スムース
float smin(float d1, float d2, float k ){
    float res = exp(-k * d1) + exp(-k * d2);
    return -log(res) / k;

    //↓softmin
    // float a=d1,b=d2;
    // float ea=exp(-a),eb=exp(-b);
    // return (a*ea+b*eb)/(ea+eb);
}


float opSmoothUnion( float d1, float d2, float k ){
    float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) - k*h*(1.0-h);
}

float opSmoothSubtraction( float d1, float d2, float k ){
    float h = clamp( 0.5 - 0.5*(d2+d1)/k, 0.0, 1.0 );
    return mix( d2, -d1, h ) + k*h*(1.0-h);
}

float opSmoothIntersection( float d1, float d2, float k ){
    float h = clamp( 0.5 - 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) + k*h*(1.0-h);
}

float twistTorus(  in vec3 p )
{
    const float k = 5.0; // or some other amount
    float c = cos(k*p.y);
    float s = sin(k*p.y);
    mat2  m = mat2(c,-s,s,c);
    vec3  q = vec3(m*p.xz,p.y);
    vec2 t=vec2(0.6,0.2);
    return sdTorus(q,t);
}

float opCheapBend_Torus( in vec3 p )
{
    p=p.zxy;
    float k = 1.0*sin(time*50*PI/180); // or some other amount
    float c = cos(k*p.x);
    float s = sin(k*p.x);
    mat2  m = mat2(c,-s,s,c);
    vec3  q = vec3(m*p.xy,p.z);
    vec2 t=vec2(0.6,0.2);
    return sdTorus(q,t);
}


vec3 bendPos(vec3 p ,float k)
{
    // p=p.zxy;
    float c = cos(k*p.x);
    float s = sin(k*p.x);
    mat2  m = mat2(c,-s,s,c);
    vec3  q = vec3(m*p.xy,p.z);
    return q;
}

float sdBox( vec3 p, vec3 b ){
  vec3 q = abs(p) - b;
  return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0);
}

vec3 hsv2rgb(vec3 c){
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

vec3 rgb2hsv(vec3 c){
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
 
    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}





// GLSL textureless classic 3D noise "cnoise",
// with an RSL-style periodic variant "pnoise".
// Author:  Stefan Gustavson (stefan.gustavson@liu.se)
// Version: 2024-11-07
//
// Many thanks to Ian McEwan of Ashima Arts for the
// ideas for permutation and gradient selection.
//
// Copyright (c) 2011 Stefan Gustavson. All rights reserved.
// Distributed under the MIT license. See LICENSE file.
// https://github.com/stegu/webgl-noise
vec3 mod289(vec3 x){return x - floor(x * (1.0 / 289.0)) * 289.0;}
vec4 mod289(vec4 x){return x - floor(x * (1.0 / 289.0)) * 289.0;}
vec4 permute(vec4 x){return mod289(((x*34.0)+10.0)*x);}
vec4 taylorInvSqrt(vec4 r){return 1.79284291400159 - 0.85373472095314 * r;}
vec3 fade(vec3 t){return t*t*t*(t*(t*6.0-15.0)+10.0);}

// Classic Perlin noise
float cnoise(vec3 P){
  vec3 Pi0 = floor(P); // Integer part for indexing
  vec3 Pi1 = Pi0 + vec3(1.0); // Integer part + 1
  Pi0 = mod289(Pi0);
  Pi1 = mod289(Pi1);
  vec3 Pf0 = fract(P); // Fractional part for interpolation
  vec3 Pf1 = Pf0 - vec3(1.0); // Fractional part - 1.0
  vec4 ix = vec4(Pi0.x, Pi1.x, Pi0.x, Pi1.x);
  vec4 iy = vec4(Pi0.yy, Pi1.yy);
  vec4 iz0 = Pi0.zzzz;
  vec4 iz1 = Pi1.zzzz;

  vec4 ixy = permute(permute(ix) + iy);
  vec4 ixy0 = permute(ixy + iz0);
  vec4 ixy1 = permute(ixy + iz1);

  vec4 gx0 = ixy0 * (1.0 / 7.0);
  vec4 gy0 = fract(floor(gx0) * (1.0 / 7.0)) - 0.5;
  gx0 = fract(gx0);
  vec4 gz0 = vec4(0.5) - abs(gx0) - abs(gy0);
  vec4 sz0 = step(gz0, vec4(0.0));
  gx0 -= sz0 * (step(0.0, gx0) - 0.5);
  gy0 -= sz0 * (step(0.0, gy0) - 0.5);

  vec4 gx1 = ixy1 * (1.0 / 7.0);
  vec4 gy1 = fract(floor(gx1) * (1.0 / 7.0)) - 0.5;
  gx1 = fract(gx1);
  vec4 gz1 = vec4(0.5) - abs(gx1) - abs(gy1);
  vec4 sz1 = step(gz1, vec4(0.0));
  gx1 -= sz1 * (step(0.0, gx1) - 0.5);
  gy1 -= sz1 * (step(0.0, gy1) - 0.5);

  vec3 g000 = vec3(gx0.x,gy0.x,gz0.x);
  vec3 g100 = vec3(gx0.y,gy0.y,gz0.y);
  vec3 g010 = vec3(gx0.z,gy0.z,gz0.z);
  vec3 g110 = vec3(gx0.w,gy0.w,gz0.w);
  vec3 g001 = vec3(gx1.x,gy1.x,gz1.x);
  vec3 g101 = vec3(gx1.y,gy1.y,gz1.y);
  vec3 g011 = vec3(gx1.z,gy1.z,gz1.z);
  vec3 g111 = vec3(gx1.w,gy1.w,gz1.w);

  vec4 norm0 = taylorInvSqrt(vec4(dot(g000, g000), dot(g010, g010), dot(g100, g100), dot(g110, g110)));
  vec4 norm1 = taylorInvSqrt(vec4(dot(g001, g001), dot(g011, g011), dot(g101, g101), dot(g111, g111)));

  float n000 = norm0.x * dot(g000, Pf0);
  float n010 = norm0.y * dot(g010, vec3(Pf0.x, Pf1.y, Pf0.z));
  float n100 = norm0.z * dot(g100, vec3(Pf1.x, Pf0.yz));
  float n110 = norm0.w * dot(g110, vec3(Pf1.xy, Pf0.z));
  float n001 = norm1.x * dot(g001, vec3(Pf0.xy, Pf1.z));
  float n011 = norm1.y * dot(g011, vec3(Pf0.x, Pf1.yz));
  float n101 = norm1.z * dot(g101, vec3(Pf1.x, Pf0.y, Pf1.z));
  float n111 = norm1.w * dot(g111, Pf1);

  vec3 fade_xyz = fade(Pf0);
  vec4 n_z = mix(vec4(n000, n100, n010, n110), vec4(n001, n101, n011, n111), fade_xyz.z);
  vec2 n_yz = mix(n_z.xy, n_z.zw, fade_xyz.y);
  float n_xyz = mix(n_yz.x, n_yz.y, fade_xyz.x); 
  return 2.2 * n_xyz;
}

// Classic Perlin noise, periodic variant
float pnoise(vec3 P, vec3 rep){
  vec3 Pi0 = mod(floor(P), rep); // Integer part, modulo period
  vec3 Pi1 = mod(Pi0 + vec3(1.0), rep); // Integer part + 1, mod period
  Pi0 = mod289(Pi0);
  Pi1 = mod289(Pi1);
  vec3 Pf0 = fract(P); // Fractional part for interpolation
  vec3 Pf1 = Pf0 - vec3(1.0); // Fractional part - 1.0
  vec4 ix = vec4(Pi0.x, Pi1.x, Pi0.x, Pi1.x);
  vec4 iy = vec4(Pi0.yy, Pi1.yy);
  vec4 iz0 = Pi0.zzzz;
  vec4 iz1 = Pi1.zzzz;

  vec4 ixy = permute(permute(ix) + iy);
  vec4 ixy0 = permute(ixy + iz0);
  vec4 ixy1 = permute(ixy + iz1);

  vec4 gx0 = ixy0 * (1.0 / 7.0);
  vec4 gy0 = fract(floor(gx0) * (1.0 / 7.0)) - 0.5;
  gx0 = fract(gx0);
  vec4 gz0 = vec4(0.5) - abs(gx0) - abs(gy0);
  vec4 sz0 = step(gz0, vec4(0.0));
  gx0 -= sz0 * (step(0.0, gx0) - 0.5);
  gy0 -= sz0 * (step(0.0, gy0) - 0.5);

  vec4 gx1 = ixy1 * (1.0 / 7.0);
  vec4 gy1 = fract(floor(gx1) * (1.0 / 7.0)) - 0.5;
  gx1 = fract(gx1);
  vec4 gz1 = vec4(0.5) - abs(gx1) - abs(gy1);
  vec4 sz1 = step(gz1, vec4(0.0));
  gx1 -= sz1 * (step(0.0, gx1) - 0.5);
  gy1 -= sz1 * (step(0.0, gy1) - 0.5);

  vec3 g000 = vec3(gx0.x,gy0.x,gz0.x);
  vec3 g100 = vec3(gx0.y,gy0.y,gz0.y);
  vec3 g010 = vec3(gx0.z,gy0.z,gz0.z);
  vec3 g110 = vec3(gx0.w,gy0.w,gz0.w);
  vec3 g001 = vec3(gx1.x,gy1.x,gz1.x);
  vec3 g101 = vec3(gx1.y,gy1.y,gz1.y);
  vec3 g011 = vec3(gx1.z,gy1.z,gz1.z);
  vec3 g111 = vec3(gx1.w,gy1.w,gz1.w);

  vec4 norm0 = taylorInvSqrt(vec4(dot(g000, g000), dot(g010, g010), dot(g100, g100), dot(g110, g110)));
  vec4 norm1 = taylorInvSqrt(vec4(dot(g001, g001), dot(g011, g011), dot(g101, g101), dot(g111, g111)));

  float n000 = norm0.x * dot(g000, Pf0);
  float n010 = norm0.y * dot(g010, vec3(Pf0.x, Pf1.y, Pf0.z));
  float n100 = norm0.z * dot(g100, vec3(Pf1.x, Pf0.yz));
  float n110 = norm0.w * dot(g110, vec3(Pf1.xy, Pf0.z));
  float n001 = norm1.x * dot(g001, vec3(Pf0.xy, Pf1.z));
  float n011 = norm1.y * dot(g011, vec3(Pf0.x, Pf1.yz));
  float n101 = norm1.z * dot(g101, vec3(Pf1.x, Pf0.y, Pf1.z));
  float n111 = norm1.w * dot(g111, Pf1);

  vec3 fade_xyz = fade(Pf0);
  vec4 n_z = mix(vec4(n000, n100, n010, n110), vec4(n001, n101, n011, n111), fade_xyz.z);
  vec2 n_yz = mix(n_z.xy, n_z.zw, fade_xyz.y);
  float n_xyz = mix(n_yz.x, n_yz.y, fade_xyz.x); 
  return 2.2 * n_xyz;
}

float rand(vec2 co){
   return fract(sin(dot(co.xy,vec2(12.9898,78.233))) * 43758.5453);
}

#define ITERATIONS 8
float deMandelbulb(vec3 p, float power) {
    vec3 z = p;
    float dr = 1.0;
    float r;
    for (int i = 0; i < ITERATIONS; i++) {
        r = length(z);
        if (r > 10.0) break;
        float theta = acos(z.y / r)+time*0.03;
        float phi = atan(z.z, z.x);
        // float phi = atan(z.z, z.x);
        // float theta = acos(z.y / r)+time*0.1+phi*0.25;
        dr = pow(r, power - 1.0) * power * dr + 1.0;

        float zr = pow(r, power);
        theta = theta * power;
        phi = phi * power;

        z = zr * vec3(sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi));
        z += p;
    }
    return 0.5 * log(r) * r / dr;
}

vec4 sdfColored(vec3 pos){
  SHADOW_RATE=1.0*0.5;
  REFLECT_NUM=10;
  SPECULAR_RATE=0.7;
  float dist=1e20;
  float ndist;

  vec3 col=vec3(0.0,0.0,0.0);

if(STAGE==0){

  vec3 torus_pos=pos;
  // mat3 col_rot=z_axis_rot(-60.0 *PI/180.0 *torus_pos.y*1.0);
  // torus_pos=torus_pos*1.4;
  torus_pos=z_axis_rot(-60.0 *PI/180.0)*torus_pos;
  // torus_pos=x_axis_rot(-10.0 *PI/180.0)*torus_pos; //←傾く!
  torus_pos=torus_pos-vec3(0.8,0.0,0.0);
  
    
  //不正でない繰り返しSDF
  //https://iquilezles.org/articles/sdfrepetition/
  float s=2.0;
  vec2 p=torus_pos.xy;
  vec2 id = round (p/s);
  vec2   o = sign (p-s*id); // 隣接オフセット方向
  dist = 1e20;
  for ( int j= 0 ; j< 2 ; j++ ){
    for ( int i= 0 ; i< 2 ; i++ ){
      vec2 rid = id + vec2 (i,j)*o;
      vec2 r = p - s*rid;
      dist = min ( dist, sdSphere(vec3(r,torus_pos.z),0.6) );
    }
  }
  
  // dist=sdTorus(torus_pos,vec2(0.6,0.2));
    // torus_pos=z_axis_rot(-60.0 *PI/180.0)*torus_pos;dist=sdBox(torus_pos,vec3(0.6,0.4,0.5));
    dist=sdSphere(torus_pos,0.6); //REFLECT_NUM=5;
    // dist=max(sdSphere(torus_pos,0.6),-sdSphere(torus_pos,0.54+0.04*sin(time*0.3))); //REFLECT_NUM=5;
    // REFLECT_NUM=5;

  //dist=sdTorus(torus_pos/0.5,vec2(0.6,0.2))*0.5; //←torus_pos=torus_pos*2.0等とするとバグる
  col=vec3(1.0,0.4,0.4);
  // col=col_rot*col;
  // dist=sdSphere(torus_pos,0.6); REFLECT_NUM=5;

  // refraction sphere
  vec3 refr_pos=pos-vec3(-2.0,1.0,0.0);
  ndist=max(sdSphere(refr_pos,0.6),-sdSphere(refr_pos,0.54+0.04*sin(time*0.3))); //REFLECT_NUM=5;
  if(ndist<dist){dist=ndist;col=REFRACTION_MATERIAL;}


  vec3 box_pos=pos;
  // [no softmin
  box_pos=box_pos-vec3(-0.55,-0.0,0.2);
  ndist=sdBox(box_pos,vec3(0.4,0.4,0.4));
  if(ndist<dist){dist=ndist;col=vec3(0.4,0.4,1.0);}
  // ]

  //[普通の合体
  // box_pos=z_axis_rot(-60.0 *PI/180.0)*box_pos;
  // box_pos=box_pos-vec3(0.8,0.0,0.0);
  // ndist=sdBox(box_pos,vec3(0.45,0.45,0.45));
  // // if(ndist<dist){dist=ndist;col=vec3(0.4,0.4,1.0);}
  // dist=max(-dist,ndist);
  //]


  // [↓スムース合体(softmin)
  // ndist=sdBox(box_pos,vec3(0.4,0.4,0.4));
  // float k=3.0+3.0*sin(time*0.4);
  // float ea=exp(k*dist),eb=exp(k*ndist);
  // dist=(dist*ea+ndist*eb)/(ea+eb);
  // col=(col*ea+vec3(0.4,0.4,1.0)*eb)/(ea+eb);
  // ]

  vec3 box2_pos=pos;
  // box2_pos=box2_pos-vec3(-0.6,1.2,0.5+0.4*sin(time*0.5));
  box2_pos=box2_pos-vec3(-0.6,1.2,0.2);
  ndist=sdBox(box2_pos,vec3(0.4,0.1,0.4));
    // box2_pos=z_axis_rot(-50.0 *PI/180.0)*box2_pos;
    // box2_pos=box2_pos-vec3(0.0,0.1,-0.1);
    // ndist=sdTorus(box2_pos,vec2(0.35,0.15));
  if(ndist<dist){dist=ndist;col=vec3(0.4,1.0,0.4);}

}


  vec3 floor_pos=pos;
  vec3 floor_size=vec3(8.0,4.0,0.1);
  floor_pos=floor_pos-vec3(-4.0,0.0,0.7);
  ndist=sdBox(floor_pos,floor_size);
  if(ndist<dist){dist=ndist;
  float kk=length(mod(floor_pos.xy,1.0)-mod(floor_pos.xy,0.5));
  col=(kk<0.4||kk>0.6)?vec3(0.8,0.8,0.8):vec3(0.4,0.4,0.4);}

  
  vec3 wall_pos=pos;
  vec3 wall_size=vec3(0.1,4.0,1.5);
  wall_pos=wall_pos-vec3(-4.0,0.0,0.7-1.5);
  ndist=sdBox(wall_pos,wall_size);
  if(ndist<dist){dist=ndist;col=NOREFLECT_MATERIAL;}


if(STAGE==1){
  vec3 fusion_pos=pos-vec3(-6.0,-3.0,0.0);
  ndist=max(sdSphere(fusion_pos,0.6), sdBox(fusion_pos,vec3(0.5,0.5,0.5)));
  if(ndist<dist){dist=ndist;col=vec3(0.7,0.7,0.7);}

  fusion_pos=pos-vec3(-6.0,-1.0,0.0);
  ndist=max(-sdSphere(fusion_pos,0.6), sdBox(fusion_pos,vec3(0.5,0.5,0.5)));
  if(ndist<dist){dist=ndist;col=vec3(0.7,0.7,0.7);}

  
  // [↓スムース合体(softmin)
  fusion_pos=pos-vec3(-6.0,1.0,0.0);
  float ndist1=sdSphere(fusion_pos-vec3(0.2,0.2,0.2),0.4);
  ndist=sdBox(fusion_pos+vec3(0.2,0.2,0.2),vec3(0.4,0.4,0.4));
  float k=3.0+3.0*sin(time*0.4);
  float ea=exp(-k*ndist1),eb=exp(-k*ndist);
  ndist=(ndist1*ea+ndist*eb)/(ea+eb);
  if(ndist<dist){dist=ndist;col=(vec3(1.0,0.4,0.4)*ea+vec3(0.4,0.4,1.0)*eb)/(ea+eb);}
  // ]

  
  // [↓スムース合体(softmin)
  fusion_pos=pos-vec3(-6.0,3.0,0.0);
  ndist1=sdSphere(fusion_pos-vec3(0.2,0.2,0.2),0.4);
  ndist=sdBox(fusion_pos+vec3(0.2,0.2,0.2),vec3(0.4,0.4,0.4));
  k=3.0+3.0*sin(time*0.4);
  ea=exp(k*ndist1),eb=exp(k*ndist);
  ndist=(ndist1*ea+ndist*eb)/(ea+eb);
  if(ndist<dist){dist=ndist;col=(vec3(1.0,0.4,0.4)*ea+vec3(0.4,0.4,1.0)*eb)/(ea+eb);}
  // ]

}
  wall_pos=pos;
  wall_size=vec3(0.1,4.0,1.5);
  wall_pos=wall_pos-vec3(-8.0,0.0,0.7-1.5);
  ndist=sdBox(wall_pos,wall_size);
  if(ndist<dist){dist=ndist;col=NOREFLECT_MATERIAL;}


if(STAGE==2){
  REFLECT_NUM=0;SHADOW_RATE=1.0;
  if(REFLECT_NUM==0&&SHADOW_RATE==1.0){
    vec3 bulb_pos=pos-vec3(vec3(-10.0,-2.0,-0.5));
    ndist=deMandelbulb(bulb_pos,8.0);
    if(ndist<dist){dist=ndist;col=vec3(1.0,1.0,0.4);}
    
    vec3 bendtorus_pos=pos-vec3(vec3(-10.0,2.0,-0.5));
    float pn=length(bendtorus_pos)-bendtorus_pos.z;
    ndist=opSmoothUnion(sdSphere(bendtorus_pos,0.5*(sin(pn*3.0+time*30  *PI/180)+1)/2),
    opCheapBend_Torus(bendtorus_pos)+sin(50*bendtorus_pos.x)*sin(50*bendtorus_pos.y)*sin(50*bendtorus_pos.z)*0.01,0.7);
    if(ndist<dist){dist=ndist;col=vec3(0.4,1.0,1.0);}
  }
}

  







  

  

  // return vec4(dist,vec3(0.8,0.8,0.8));
  return vec4(dist,col);

}

/* return:vec4(dist,col.xyz)
vec4 sdfColored_tmp(vec3 pos){
  SHADOW_RATE=1.0*0.5;
  REFLECT_NUM=5;
  // REFRACTION_NUM=0;
  float dist=1e20;

  
  // ↓座標の方を曲げているので繰り返しの誤実装同様正しく描画されない
  // SHADOW_RATE=0.5;
  // REFLECT_NUM=0;
  // SPECULAR_RATE=0;
  // int k=5;
  // float dist=1e20;
  // for(float i=1.0; i<40.0; i*=2.0){
  //   float s=50.0/i;
  //   // pos.z+=pnoise(vec3(pos.xy+i,0.0),vec3(s,s,s))*s*0.01;
  //   // pos.z+=(0.5+pnoise(vec3(pos.xy*i+i,0.0),vec3(s,s,s)))*s*0.01;
  //   float a=1.0;
  //   if(time*3.0<k)a=clamp(time*3.0+1-k,0.0,1.0);
  //   pos.z+=(0.5+pnoise(vec3(pos.xy*i+i,0.0),vec3(s,s,s)))*s*0.01*a;
  //   // pos.z+=pNoise(pos.xy,s);
  //   k+=5;
  // }
  // return vec4(-pos.z+0.5,0.4,0.7,0.4);
  
  // return vec4(dist-pos.z,0.4,0.7,0.4);



  // pos=z_axis_rot(time*0.5)*pos;

  vec3 col=vec3(0.0,0.0,0.0);

  vec3 torus_pos=pos;
  // mat3 col_rot=z_axis_rot(-60.0 *PI/180.0 *torus_pos.y*1.0);
  // torus_pos=torus_pos*1.4;
  torus_pos=z_axis_rot(-60.0 *PI/180.0)*torus_pos;
  // torus_pos=x_axis_rot(-10.0 *PI/180.0)*torus_pos; //←傾く!
  torus_pos=torus_pos-vec3(0.8,0.0,0.0);
  
    // float s=1.8;
    // s=(abs(torus_pos.z)<1.0?s:10000.0); //←1段分に
    // torus_pos = torus_pos - s*floor(torus_pos/s+0.5);
    // // ↑単純にtorus_pos=mod(torus_pos,4.0)とするとバグる floor(x+0.5)はround(x)
    // // mod処理が最後じゃないとバグる
    // // ↑何やってもバグる
    // // modは単体だとバグらないが、周囲の物体との位置関係によりバグる
    // float s=2.0;
    // torus_pos=mod(torus_pos+vec3(s,s,s)/2.0,vec3(s,s,10000.0))-vec3(s,s,s)/2.0;

    // torus_pos=abs(torus_pos.z)<1.0?repeat_mirrored_3d(torus_pos,2.0):torus_pos; 
    // //↑他の物体と干渉するとバグる
    
  //不正でない繰り返しSDF
  //https://iquilezles.org/articles/sdfrepetition/
  float s=2.0;
  vec2 p=torus_pos.xy;
  vec2 id = round (p/s);
  vec2   o = sign (p-s*id); // 隣接オフセット方向
  dist = 1e20;
  for ( int j= 0 ; j< 2 ; j++ ){
    for ( int i= 0 ; i< 2 ; i++ ){
      vec2 rid = id + vec2 (i,j)*o;
      vec2 r = p - s*rid;
      dist = min ( dist, sdSphere(vec3(r,torus_pos.z),0.6) );
    }
  }
  
  // dist=sdTorus(torus_pos,vec2(0.6,0.2));
    // dist=sdSphere(torus_pos,0.6); //REFLECT_NUM=5;
    dist=max(sdSphere(torus_pos,0.6),-sdSphere(torus_pos,0.54+0.04*sin(time*0.3))); //REFLECT_NUM=5;
    // REFLECT_NUM=5;

  //dist=sdTorus(torus_pos/0.5,vec2(0.6,0.2))*0.5; //←torus_pos=torus_pos*2.0等とするとバグる
  col=vec3(1.0,0.4,0.4);
  // col=col_rot*col;
  // dist=sdSphere(torus_pos,0.6); REFLECT_NUM=5;

  vec3 box_pos=pos;
  float ndist;
  // [no softmin
  box_pos=box_pos-vec3(-0.55,-0.0,0.2);
  ndist=sdBox(box_pos,vec3(0.4,0.4,0.4));
  if(ndist<dist){dist=ndist;col=vec3(0.4,0.4,1.0);}
  // ]

  // [↓スムース合体(softmin)
  // ndist=sdBox(box_pos,vec3(0.4,0.4,0.4));
  // float k=3.0+3.0*sin(time*0.4);
  // float ea=exp(-k*dist),eb=exp(-k*ndist);
  // dist=(dist*ea+ndist*eb)/(ea+eb);
  // col=(col*ea+vec3(0.4,0.4,1.0)*eb)/(ea+eb);
  // ]

  vec3 box2_pos=pos;
  // box2_pos=box2_pos-vec3(-0.6,1.2,0.5+0.4*sin(time*0.5));
  box2_pos=box2_pos-vec3(-0.6,1.2,0.2);
  ndist=sdBox(box2_pos,vec3(0.4,0.1,0.4));
    // box2_pos=z_axis_rot(-50.0 *PI/180.0)*box2_pos;
    // box2_pos=box2_pos-vec3(0.0,0.1,-0.1);
    // ndist=sdTorus(box2_pos,vec2(0.35,0.15));
  if(ndist<dist){dist=ndist;col=vec3(0.4,1.0,0.4);}

  // ndist=deMandelbulb(pos-vec3(-3.0,0.0,0.0),8.0);
  // if(ndist<dist){dist=ndist;col=vec3(1.0,1.0,0.4);}

  vec3 floor_pos=pos;
  vec3 floor_size=vec3(4.0,4.0,0.1);
  floor_pos=floor_pos-vec3(0.0,0.0,0.7);
  ndist=sdBox(floor_pos,floor_size);
  if(ndist<dist){dist=ndist;
  float kk=length(mod(floor_pos.xy,1.0)-mod(floor_pos.xy,0.5));
  col=(kk<0.4||kk>0.6)?vec3(0.8,0.8,0.8):vec3(0.4,0.4,0.4);}

  // return vec4(dist,vec3(1.0,1.0,1.0));
  return vec4(dist,col);

  //return sdBox(pos,vec3(1.0,1.0,1.0));
  // return de(pos);
  //return max(sdSphere(pos), sdTorus(pos));

  // float pn=length(pos);
  // vec3 p = mod(pos, 8.0) - 4.0;

  //return opCheapBend_Torus(p);

  // return opSmoothUnion(sdSphere(p,1.5*(sin(pn+time*30  *PI/180)+1)/2),
  // opCheapBend_Torus(p)+sin(20*p.x)*sin(20*p.y)*sin(20*p.z)*0.02,0.7);
}//*/
vec3 sdfNorm(vec3 p){
  return normalize(vec3(
  sdfColored(p+DX).x-sdfColored(p-DX).x,
  sdfColored(p+DY).x-sdfColored(p-DY).x,
  sdfColored(p+DZ).x-sdfColored(p-DZ).x
  ));
}

float TOTAL_DIST_MAX=10000.0;
bool rayMarch(inout vec3 pos,inout vec3 dir,out vec3 normal,
  in vec3 light_dir,in vec4 ambient, out vec4 color, inout float total_dist,
  out vec3 refraction_dir, in bool isInObject){
  vec3 pos0=pos;
  bool hitflag=false;
  // 5. Rayの判定
  float t = 0.0, d;
  for (int i = 0; i < 128; i++ ){ //何回でもok 十分な数
    d = sdfColored(pos).x; //距離関数から現在の距離を求める //回転行列をかける
    if(isInObject)d=-d; //屈折時物体の中
    if (d < MARCHING_DELTA) {  //計算できた距離が十分に０に近かったら
      hitflag=true;
      break; //衝突したという判定
    } 
    t += d; //当たらなかったら
    pos = pos0 + t * dir; //rayの座標更新 どれだけrayを進めるかというと最も近いオブジェクトまでの距離（突き抜け防止
    total_dist += d;

    //total_dist(反射も含めた総距離)を制限して描画を軽くする
    if(total_dist>TOTAL_DIST_MAX){
      hitflag=false;
      break;
    }
  }
  normal=sdfNorm(pos);
  if(isInObject)normal=-normal;


  float l = dot(normal, -light_dir); //法線ベクトルと光源のベクトルの内積（dot)
  l=clamp(l,0.0,1.0);
  vec3 mat_color=sdfColored(pos).yzw;

  // vec3 reflect_ray=(-ray.dir)+2*(normal-(-ray.dir)); //レイが反射していく方向
  float cos_theta1=dot(normal,-dir);
  float sin_theta1=sqrt(1-cos_theta1*cos_theta1);
  vec3 reflect_ray=normalize(2.0*normal*cos_theta1+dir); //レイが反射していく方向
  vec3 reflect_light=normalize(2.0*normal*dot(normal,-light_dir)+light_dir); //光の反射方向

  vec3 dir_vertical=-normal*cos_theta1; //面に対して垂直方向
  vec3 dir_parallel=dir-dir_vertical; //面に対して水平方向
  float sin_theta2=sin_theta1/REFRACTION_INDEX;
    if(isInObject)sin_theta2=sin_theta1*REFRACTION_INDEX;
    // if(sin_theta2>1)hitflag=false; //全反射の判定
  float cos_theta2=sqrt(1-sin_theta2*sin_theta2);
  refraction_dir=normalize(dir_vertical*cos_theta2/cos_theta1
    +dir_parallel*sin_theta2/sin_theta1);
  if(sin_theta2>1)refraction_dir=reflect_ray; //全反射の判定
  


  float spec_dot=dot(reflect_light,-dir);
  // spec_dot=clamp(spec_dot,0.0,1.0);
  spec_dot=max(spec_dot,0.01);
  float specular=pow(spec_dot,20.0);

  color = vec4(l*mat_color,1.0)+
    vec4(ambient.xyz*mat_color,1.0)+
    vec4(specular,specular,specular,1.0)*SPECULAR_RATE;

  if(mat_color==REFRACTION_MATERIAL)color=vec4(mat_color,1.0); //REFRACTION TODO
    
  // if(isInObject)color=vec4(mat_color,1.0); //TODO

  if(!hitflag){ //空色
    vec3 upper_dir=vec3(0.0,0.0,1.0);
    upper_dir=light_dir;
    vec4 skycol_top=vec4(0.8,1.0,1.0,1.0),skycol_bottom=vec4(0.3,0.6,0.6,1.0);
    float lookheight=dot(upper_dir,-dir)*0.5+0.5;
    color=skycol_top*lookheight+skycol_bottom*(1.0-lookheight);
  }

  
    vec3 color_hsv=rgb2hsv(color.xyz);
    if(hitflag)color=vec4(hsv2rgb(
      vec3(color_hsv.x,color_hsv.y*max(1.0-150.0*total_dist/TOTAL_DIST_MAX,0.0),color_hsv.z)),1.0);


  dir=reflect_ray; //反射したベクトルを次のレイの向きに

  return hitflag;
}


void main (void){
  gl_FragColor = vec4(0); //背景色
  vec2 pos = ( gl_FragCoord.xy * 2.0 - resolution) / max(resolution.x, resolution.y);
  //gl_FragColor = vec4(pos, 0.0, 1.0); //最終的な色の描画

  vec2 rot_angle=vec2(mouse_pos.x/resolution.x*2*PI,0*PI+mouse_pos.y/resolution.y*2*PI);
  mat3 rot_mouse = z_axis_rot(-rot_angle.x)
   * x_axis_rot(-rot_angle.y);
  
  vec2 rot_angle_c=vec2(cursor_pos.x*2*PI,0*PI+cursor_pos.y*2*PI);
  // mat3 rot_cursor = z_axis_rot(-rot_angle_c.x)
  //  * x_axis_rot(-rot_angle_c.y);
  mat3 rot_cursor = z_axis_rot(-rot_angle_c.x-0.75*PI)
   * x_axis_rot(-rot_angle_c.y-(PI+0.65*PI));

  mat3 rot = z_axis_rot(-0.75*PI)
   * x_axis_rot(-(PI+0.65*PI));
  
  // rot=rot_cursor;

  // rot_angle=vec2((mouse_pos.x/resolution.x-0.5)*2.0*PI,-0.5*PI+(mouse_pos.y/resolution.y-0.5)*PI);
  // rot_mouse = z_axis_rot(-rot_angle.x) * x_axis_rot(-rot_angle.y);
  rot_mouse = z_axis_rot(-camera_ang.x) * x_axis_rot(-camera_ang.y);
  // rot=rot_mouse*rot;
  rot=rot_mouse;

  //2. 始点の定義 (カメラの姿勢が定まる
  vec3 camera_pos = vec3(0.0, 0.0, -4.0 + time*1.0); //カメラの位置
  camera_pos = vec3(-1.5, 1.7, -1.4);
  camera_pos = CameraPos;
  if(camera_pos.x>-4.0)STAGE=0;
  else if(camera_pos.x>-8.0)STAGE=1;
  else STAGE=2;

  vec3 camera_up = vec3(0.0, 1.0, 0.0);  //カメラの上向きベクトル
  vec3 camera_dir = vec3(0.0, 0.0, 1.0); //カメラの前向きベクトル
  vec3 camera_side = cross(camera_up, camera_dir); //カメラの横向きベクトル (上向きベクトルと前向きベクトルの外積
  camera_up=rot*camera_up;
  camera_dir=rot*camera_dir;
  camera_side=rot*camera_side;


  //8. 光源が当たる方向を定義(z方向に光があたるように
  vec3 light_dir = normalize(vec3(-0.3,-0.6, 1.0)); //マウスを光源とする
  light_dir=normalize(z_axis_rot(time*0.03)*light_dir);
  light_dir=normalize(vec3(-0.3+0.5,-0.6, 1.0)); //TODO
  light_dir=normalize(vec3(-0.3,-0.6, 1.0)); //TODO
  // light_dir = normalize(rot_cursor*normalize(vec3(-0.0,-0.0, 1.0)));

  // 4. Rayの設定
  Ray ray; //ここはインスタンスか
  ray.pos = camera_pos; //Rayの初期位置
  ray.dir = normalize(pos.x * camera_side + pos.y * camera_up + camera_dir); // Rayの進行方向はカメラの姿勢から求めることができる

  vec3 normal;
  vec4 ambient=vec4(0.4,0.4,0.4,1.0);
  // ambient=vec4(0.2,0.2,0.2,1.0); //TODO
  vec4 color;
  float total_dist=0.0;
  vec3 refraction_dir;
  bool hitflag=rayMarch(ray.pos,ray.dir,normal,
    light_dir,ambient,color,total_dist,
    refraction_dir,false);




  // if (hitflag) {
  //     gl_FragColor=color;

  //     ray.pos=ray.pos+ray.dir*MARCHING_DELTA*2.0; //TODO:ここの2.0調整
  //     hitflag=rayMarch(ray.pos,ray.dir,normal,
  //       light_dir,ambient,color);
      
  //     if(hitflag)gl_FragColor=gl_FragColor*0.5+0.5*color;
  // }
  vec4 first_color=color;
  if(hitflag){
    gl_FragColor=color;

    if(dot(normal,-light_dir)>0.0||sdfColored(ray.pos).yzw==REFRACTION_MATERIAL){//TODO:自分自身への影が濃すぎる?
    vec4 tmp_col;
    vec3 tmp_pos=ray.pos,tmp_dir=-light_dir,tmp_norm=normal;
    float tmp_total_dist=total_dist;
      tmp_pos=tmp_pos+normal*MARCHING_DELTA*100.0; //ここを2.0→10.0にしたら直った? 状況次第で100.0
    vec3 tmp_pos_first=tmp_pos;
    vec3 tmp_refl;
    bool isInShadow=rayMarch(tmp_pos,tmp_dir,tmp_norm,
    light_dir,ambient,tmp_col,tmp_total_dist,
    tmp_refl,false);

    while(isInShadow&&sdfColored(tmp_pos).yzw==REFRACTION_MATERIAL){ //isInShadow&& がないと黒い粉がたくさん出てくる!
        tmp_dir=tmp_refl;
        tmp_pos=tmp_pos+tmp_dir*MARCHING_DELTA*100.0; // ここは100.0だと変になる
        isInShadow=rayMarch(tmp_pos,tmp_dir,tmp_norm,
          light_dir,ambient,tmp_col,tmp_total_dist,
          tmp_refl,true);
        
        // if(!isInShadow)break;
        // if(isInShadow){

        tmp_dir=tmp_refl;
        tmp_pos=tmp_pos+tmp_dir*MARCHING_DELTA*100.0; // ここは100.0だと変になる
        isInShadow=rayMarch(tmp_pos,tmp_dir,tmp_norm,
          light_dir,ambient,tmp_col,tmp_total_dist,
          tmp_refl,false);
        // }
        
      }


    if(isInShadow
    // &&sdfColored(ray.pos).yzw!=sdfColored(tmp_pos).yzw
    // ||sdfColored(tmp_pos_first).x<0.0
    )gl_FragColor*=SHADOW_RATE;
    // gl_FragColor*=max(dot(tmp_norm,-light_dir)+1.0,0.0);
    }else{gl_FragColor*=SHADOW_RATE;}
    //TODO 透明な物体での影の計算を修正
    
    // if(sdfColored(ray.pos).yzw==REFRACTION_MATERIAL){
    // for (int i=0; i<REFRACTION_NUM; i++) {
    //   ray.dir=refraction_dir;
    //   ray.pos=ray.pos+ray.dir*MARCHING_DELTA*10.0; // ここは100.0だと変になる
    //   hitflag=rayMarch(ray.pos,ray.dir,normal,
    //     light_dir,ambient,color,total_dist,
    //     refraction_dir,true);
      
    //   if(!hitflag)break;

    //   ray.dir=refraction_dir;
    //   ray.pos=ray.pos+ray.dir*MARCHING_DELTA*10.0; // ここは100.0だと変になる
    //   hitflag=rayMarch(ray.pos,ray.dir,normal,
    //     light_dir,ambient,color,total_dist,
    //     refraction_dir,false);
      
    //   if(hitflag||true)gl_FragColor=gl_FragColor*((1.0-REFRACTION_RATE)+REFRACTION_RATE*color)*0+color; //TODO
    //   else break;

    //   if(!hitflag)break;
      
    //   if(sdfColored(ray.pos).yzw!=REFRACTION_MATERIAL)break;
    // }
    // }


    // // REFLECT_NUM=5;
    // for (int i=0; i<REFLECT_NUM; i++) {
    //   ray.pos=ray.pos+ray.dir*MARCHING_DELTA*100.0; //TODO:ここの2.0調整 状況次第で100.0
    //   hitflag=rayMarch(ray.pos,ray.dir,normal,
    //     light_dir,ambient,color,total_dist,
    //     refraction_dir,false);
      
    //   if(hitflag||true)gl_FragColor=gl_FragColor*((1.0-REFLECT_RATE)+REFLECT_RATE*color);
    //   else break;

    //   if(!hitflag)break;
    // }




    
    for (int i=0; i<REFLECT_NUM; i++) {
      
      if(sdfColored(ray.pos).yzw==REFRACTION_MATERIAL){
        ray.dir=refraction_dir;
        ray.pos=ray.pos+ray.dir*MARCHING_DELTA*10.0; // ここは100.0だと変になる
        hitflag=rayMarch(ray.pos,ray.dir,normal,
          light_dir,ambient,color,total_dist,
          refraction_dir,true);
        
        if(!hitflag)break;

        ray.dir=refraction_dir;
        ray.pos=ray.pos+ray.dir*MARCHING_DELTA*10.0; // ここは100.0だと変になる
        hitflag=rayMarch(ray.pos,ray.dir,normal,
          light_dir,ambient,color,total_dist,
          refraction_dir,false);
        
        if(hitflag||true)gl_FragColor=gl_FragColor*((1.0-REFRACTION_RATE)+REFRACTION_RATE*color); //TODO
        else break;

        if(!hitflag)break;
        
        // if(sdfColored(ray.pos).yzw!=REFRACTION_MATERIAL)break;
      }else if(sdfColored(ray.pos).yzw!=NOREFLECT_MATERIAL){
        ray.pos=ray.pos+ray.dir*MARCHING_DELTA*100.0; //TODO:ここの2.0調整 状況次第で100.0
        hitflag=rayMarch(ray.pos,ray.dir,normal,
          light_dir,ambient,color,total_dist,
          refraction_dir,false);
        
        if(hitflag||true)gl_FragColor=gl_FragColor*((1.0-REFLECT_RATE)+REFLECT_RATE*color);
        else break;

        if(!hitflag)break;
      }
    
    }











  }else{ //空色
    gl_FragColor=color;
  }
}
