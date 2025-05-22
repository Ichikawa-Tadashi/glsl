#version 120

// simple.frag

#define MARCHING_DELTA 0.001

#define PI 3.14159265359

varying vec4 position;
varying vec3 normal;

uniform vec2 resolution;
uniform vec2 mouse_pos;
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
/*
vec3 sdSphere_norm(vec3 p){
  return normalize(vec3(
  sdSphere(p+DX)-sdSphere(p-DX),
  sdSphere(p+DY)-sdSphere(p-DY),
  sdSphere(p+DZ)-sdSphere(p-DZ)
  ));
}
*/

float sdTorus( vec3 p ){
  vec2 t=vec2(1.5,0.6);
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}
vec3 sdTorus_norm(vec3 p){
  return normalize(vec3(
  sdTorus(p+DX)-sdTorus(p-DX),
  sdTorus(p+DY)-sdTorus(p-DY),
  sdTorus(p+DZ)-sdTorus(p-DZ)
  ));
}

// 16. スムース
float smin(float d1, float d2, float k ){
    float res = exp(-k * d1) + exp(-k * d2);
    return -log(res) / k;
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
    return sdTorus(q);
}

float opCheapBend_Torus( in vec3 p )
{
    p=p.zxy;
    float k = 0.4*sin(time*50*PI/180); // or some other amount
    float c = cos(k*p.x);
    float s = sin(k*p.x);
    mat2  m = mat2(c,-s,s,c);
    vec3  q = vec3(m*p.xy,p.z);
    return sdTorus(q);
}

float sdBox( vec3 p, vec3 b ){
  vec3 q = abs(p) - b;
  return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0);
}


float rand(vec2 c){
	return fract(sin(dot(c.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

float noise(vec2 p, float freq ){
	float unit = resolution.x/freq;
	vec2 ij = floor(p/unit);
	vec2 xy = mod(p,unit)/unit;
	//xy = 3.*xy*xy-2.*xy*xy*xy;
	xy = .5*(1.-cos(PI*xy));
	float a = rand((ij+vec2(0.,0.)));
	float b = rand((ij+vec2(1.,0.)));
	float c = rand((ij+vec2(0.,1.)));
	float d = rand((ij+vec2(1.,1.)));
	float x1 = mix(a, b, xy.x);
	float x2 = mix(c, d, xy.x);
	return mix(x1, x2, xy.y);
}

float pNoise(vec2 p, int res){
	float persistance = .5;
	float n = 0.;
	float normK = 0.;
	float f = 4.;
	float amp = 1.;
	int iCount = 0;
	for (int i = 0; i<50; i++){
		n+=amp*noise(p, f);
		f*=2.;
		normK+=amp;
		amp*=persistance;
		if (iCount == res) break;
		iCount++;
	}
	float nf = n/normK;
	return nf*nf*nf*nf;
}



//
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
//

vec3 mod289(vec3 x)
{
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 mod289(vec4 x)
{
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x)
{
  return mod289(((x*34.0)+10.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

vec3 fade(vec3 t) {
  return t*t*t*(t*(t*6.0-15.0)+10.0);
}

// Classic Perlin noise
float cnoise(vec3 P)
{
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
float pnoise(vec3 P, vec3 rep)
{
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

// demo code:
// float color(vec2 xy) { return cnoise(vec3(1.5*xy, 0.3*iTime)); }
// void mainImage(out vec4 fragColor, in vec2 fragCoord) {
//     vec2 p = (fragCoord.xy/iResolution.y) * 2.0 - 1.0;

//     vec3 xyz = vec3(p, 0);

//     vec2 step = vec2(1.3, 1.7);
//     float n = color(xyz.xy);
//     n += 0.5 * color(xyz.xy * 2.0 - step);
//     n += 0.25 * color(xyz.xy * 4.0 - 2.0 * step);
//     n += 0.125 * color(xyz.xy * 8.0 - 3.0 * step);
//     n += 0.0625 * color(xyz.xy * 16.0 - 4.0 * step);
//     n += 0.03125 * color(xyz.xy * 32.0 - 5.0 * step);

//     fragColor.xyz = vec3(0.5 + 0.5 * vec3(n, n, n));

// }




float object_d(vec3 pos){
  //return sdBox(pos,vec3(1.0,1.0,1.0));
  //return max(sdSphere(pos), sdTorus(pos));

  // float ans=0;
  // for(float i=1; i<8; i*=2){
  //   float s=50.0*i;
  //   ans+=pnoise(pos,vec3(s,s,s));
  // }
  // return ans-pos.z;

  // pos.z+=pnoise(vec3(pos.xy,0.0),vec3(10.0,10.0,10.0));
  
  int k=5;
  for(float i=1.0; i<80.0; i*=2.0){
    float s=50.0/i;
    // pos.z+=pnoise(vec3(pos.xy+i,0.0),vec3(s,s,s))*s*0.01;
    // pos.z+=(0.5+pnoise(vec3(pos.xy*i+i,0.0),vec3(s,s,s)))*s*0.01;
    float a=1.0;
    if(time*3.0<k)a=clamp(time*3.0+1-k,0.0,1.0);
    pos.z+=(0.5+pnoise(vec3(pos.xy*i+i,0.0),vec3(s,s,s)))*s*0.01*a;
    // pos.z+=pNoise(pos.xy,s);
    k+=5;
  }

  return -pos.z+1.0;

  return pnoise(pos,vec3(10.0,10.0,10.0))-pos.z;
  float pn=length(pos);
  vec3 p = mod(pos, 8.0) - 4.0;
  //return opCheapBend_Torus(p);
  return opSmoothUnion(sdSphere(p,1.5*(sin(pn+time*30  *PI/180)+1)/2),
  opCheapBend_Torus(p)+sin(20*p.x)*sin(20*p.y)*sin(20*p.z)*0.02,0.7);
}
vec3 object_norm(vec3 p){
  return normalize(vec3(
  object_d(p+DX)-object_d(p-DX),
  object_d(p+DY)-object_d(p-DY),
  object_d(p+DZ)-object_d(p-DZ)
  ));
}



void main (void){
  vec2 pos = ( gl_FragCoord.xy * 2.0 - resolution) / max(resolution.x, resolution.y);
  //gl_FragColor = vec4(pos, 0.0, 1.0); //最終的な色の描画

  vec2 rot_angle=vec2(mouse_pos.x/resolution.x*PI,PI+mouse_pos.y/resolution.y*PI);
  //13. 回転行列追加
  mat3 rot = z_axis_rot(-rot_angle.x)
   * x_axis_rot(-rot_angle.y);

  //2. 始点の定義 (カメラの姿勢が定まる
  vec3 camera_pos = vec3(0.0, 0.0, -4.0 + time*1.0); //カメラの位置
  camera_pos = vec3(0.0,0.0,-0.0);
  vec3 camera_up = vec3(0.0, 1.0, 0.0);  //カメラの上向きベクトル
  vec3 camera_dir = vec3(0.0, 0.0, 1.0); //カメラの前向きベクトル
  vec3 camera_side = cross(camera_up, camera_dir); //カメラの横向きベクトル (上向きベクトルと前向きベクトルの外積
  // camera_up=rot*camera_up;
  // camera_dir=rot*camera_dir;
  // camera_side=rot*camera_side;

  // 4. Rayの設定
  Ray ray; //ここはインスタンスか
  ray.pos = camera_pos; //Rayの初期位置
  ray.dir = normalize(pos.x * camera_side + pos.y * camera_up + camera_dir); // Rayの進行方向はカメラの姿勢から求めることができる


  ray.pos=rot* ray.pos;
  ray.dir=rot* ray.dir;
  vec3 camera_pos_rot=rot* camera_pos;
  // 5. Rayの判定
  float t = 0.0, d;
  for (int i = 0; i < 128; i++ ){ //何回でもok 十分な数
    d = object_d(ray.pos); //距離関数から現在の距離を求める //回転行列をかける
    if (d < MARCHING_DELTA) {  //計算できた距離が十分に０に近かったら
        break; //衝突したという判定
    } 
    t += d; //当たらなかったら
    ray.pos = camera_pos_rot + t * ray.dir; //rayの座標更新 どれだけrayを進めるかというと最も近いオブジェクトまでの距離（突き抜け防止
  }

   //8. 光源が当たる方向を定義(z方向に光があたるように
   vec3 light_dir = normalize(vec3(-0.3,-0.6, 1.0)); //マウスを光源とする
   vec3 normal = object_norm(ray.pos);

   float l = dot(normal, - light_dir); //法線ベクトルと光源のベクトルの内積（dot)
  l=clamp(l,0.0,1.0);
  //6. あたったら色を変える 
  //9, 色を内積の値にする
  vec4 ambient=vec4(0.4,0.4,0.4,1.0),maxcolor=vec4(1.0,1.0,1.0,1.0);
  if (d < MARCHING_DELTA) {
      // gl_FragColor =ambient + vec4(l, l, l, 1.0) * (maxcolor-ambient);
      // gl_FragColor =maxcolor-(maxcolor-ambient)*(maxcolor-vec4(l, l, l, 1.0));
      float ll=0.1+l*0.9;
      gl_FragColor =vec4(ll, ll, ll, 1.0);
  } else {
      gl_FragColor = vec4(1.0,0.0,0.0,1.0);
  }
}
