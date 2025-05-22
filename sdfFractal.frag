#version 120

// simple.frag

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



float sdCross(vec3 p, float c) {
    p = abs(p);
    float dxy = max(p.x, p.y);
    float dyz = max(p.y, p.z);
    float dxz = max(p.x, p.z);
    return min(dxy, min(dyz, dxz)) - c;
}

/*
#define ITERATIONS 5
float deMengerSponge1(vec3 p, float scale, float width) {
    float d = sdBox(p, vec3(1.0));
    float s = 1.0;
    for (int i = 0; i < ITERATIONS; i++) {
        vec3 a = mod(p * s, 2.0) - 1.0;
        s *= scale;
        vec3 r = 1.0 - scale * abs(a);
        float c = sdCross(r, width) / s;
        d = max(d, c);
    }
    return d;
}
*/

// vec4 qmul(vec4 a, vec4 b) {
//     return vec4(
//         a.x * b.x - a.y * b.y - a.z * b.z - a.w * b.w,
//         a.x * b.y + a.y * b.x - a.z * b.w + a.w * b.z,
//         a.x * b.z + a.y * b.w + a.z * b.x - a.w * b.y,
//         a.x * b.w - a.y * b.z + a.z * b.y + a.w * b.x
//     );
// }


// #define ITERATIONS 16
// float deQuaternionMandelbrot(vec4 p) {
//     vec4 z = vec4(0.0);
//     vec4 dz = vec4(0.0);
//     vec4 pz, pdz;
//     float r, dr;

//   //   mat3 rot = z_axis_rot(0.5);
//   //  //* x_axis_rot(-0.2);
//   //   vec3 oawdj=vec3(0.01,0.02,0.0);

//     for (int i = 0; i < ITERATIONS; i++) {
//         pz = z;
//         pz.x=(pz.x<0?-pz.x:pz.x);
//         pz.y=(pz.y<0?-pz.y:pz.y);
//         // pz.z=(pz.z<0?-pz.z:pz.z);
//         z = qmul(pz, pz) + p;
//         pdz = dz;
//         dz = 2.0 * qmul(pz, pdz) + 1.0;
//         r = length(z);
//         dr = length(dz);
//         if (r > 8.0)  break;
//     }
//     return 0.5 * log(r) * r / dr;
// }


// float de(vec3 p) {
//     return deQuaternionMandelbrot(vec4(p - vec3(0.5, 0.0, 0.0), 0.0));
// }



#define ITERATIONS 8
float deMandelbulb(vec3 p, float power) {
    vec3 z = p;
    float dr = 1.0;
    float r;
    for (int i = 0; i < ITERATIONS; i++) {
        r = length(z);
        if (r > 10.0) break;
        float theta = acos(z.y / r)+time*0.1;
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
float de(vec3 p) {
    return deMandelbulb(p, 8.0);
}

vec4 qmul(vec4 a, vec4 b) {
    return vec4(
        a.x * b.x - a.y * b.y - a.z * b.z - a.w * b.w,
        a.x * b.y + a.y * b.x - a.z * b.w + a.w * b.z,
        a.x * b.z + a.y * b.w + a.z * b.x - a.w * b.y,
        a.x * b.w - a.y * b.z + a.z * b.y + a.w * b.x
    );
}


// #define ITERATIONS 16
// float deQuaternionJuliaSet(vec4 p, vec4 c) {
//     vec4 z = p;
//     vec4 dz = vec4(1.0, 0.0, 0.0, 0.0);
//     vec4 pz, pdz;
//     float r = 0.0, dr = 1.0;
//     for (int i = 0; i < ITERATIONS; i++) {
//         pz = z;
//         z = qmul(pz, pz) + c;
//         pdz = dz;
//         dz = 2.0 * qmul(pz, pdz);
//         r = length(z);
//         dr = length(dz);
//         if (r > 4.0) break;
//     }
//     return 0.5 * log(r) * r / dr;
// }


// float de(vec3 p) {
//     return deQuaternionJuliaSet(vec4(p, 0.0), vec4(-1.0, 0.2, 0.0, 0.0));
// }




// // ref: https://docs.google.com/presentation/d/1j4t4mcLw8F1PfqvKP3P8meMJ5dWfDXkfb9lc63qOFVM/edit#slide=id.g2460f5a976_0_0
// #define ITERATIONS 8
// float dePseudoKleinian(vec3 p) {
//     vec3 csize = vec3(0.90756, 0.92436, 0.90756);
//     float size = 1.0;
//     vec3 c = vec3(0.0);
//     float defactor = 1.0;
//     vec3 offset = vec3(0.0);
//     vec3 ap = p + 1.0;
//     for (int i = 0; i < ITERATIONS; i++) {
//         ap = p;
//         p = 2.0 * clamp(p, -csize, csize) - p;
//         float r2 = dot(p, p);
//         float k = max(size / r2, 1.0);
//         p *= k;
//         defactor *= k;
//         p += c;
//     }
//     float r = abs(0.5 * abs(p.y - offset.y) / defactor);
//     return r;
// }

// float de(vec3 p) {
//     return dePseudoKleinian(p);
// }


float object_d(vec3 pos){
  //return sdBox(pos,vec3(1.0,1.0,1.0));
  return de(pos);
  //return max(sdSphere(pos), sdTorus(pos));
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
  camera_pos = vec3(0.0,0.0,-1.5);
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
  for (int i = 0; i < 128*8; i++ ){ //何回でもok 十分な数
    d = object_d(ray.pos); //距離関数から現在の距離を求める //回転行列をかける
    if (d < 0.0001) {  //計算できた距離が十分に０に近かったら
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
  if (d < 0.0001) {
      // gl_FragColor =ambient + vec4(l, l, l, 1.0) * (maxcolor-ambient);
      // gl_FragColor =maxcolor-(maxcolor-ambient)*(maxcolor-vec4(l, l, l, 1.0));
      float ll=0.1+l*0.9;
      gl_FragColor =vec4(ll, ll, ll, 1.0);
  } else {
      gl_FragColor = vec4(1.0,0.0,0.0,1.0);
  }
}
