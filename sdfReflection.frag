#version 130

// simple.frag

#define PI 3.14159265359
#define MARCHING_DELTA 0.001
int REFLECT_NUM=10*0;
float SHADOW_RATE=0.5;

varying vec4 position;
varying vec3 normal;

uniform vec2 resolution;
uniform vec2 mouse_pos;
uniform vec2 cursor_pos;
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

float sdTorus( vec3 p , vec2 t){
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}
// vec3 sdTorus_norm(vec3 p){
//   return normalize(vec3(
//   sdTorus(p+DX)-sdTorus(p-DX),
//   sdTorus(p+DY)-sdTorus(p-DY),
//   sdTorus(p+DZ)-sdTorus(p-DZ)
//   ));
// }

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
    vec2 t=vec2(1.5,0.6);
    return sdTorus(q,t);
}

float opCheapBend_Torus( in vec3 p )
{
    p=p.zxy;
    float k = 0.4*sin(time*50*PI/180); // or some other amount
    float c = cos(k*p.x);
    float s = sin(k*p.x);
    mat2  m = mat2(c,-s,s,c);
    vec3  q = vec3(m*p.xy,p.z);
    vec2 t=vec2(1.5,0.6);
    return sdTorus(q,t);
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


// vec4(dist,col.xyz)
vec4 sdfColored(vec3 pos){
  // pos=z_axis_rot(time*0.5)*pos;

  float dist;
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
  dist = 1e20 ;
  for ( int j= 0 ; j< 2 ; j++ ){
    for ( int i= 0 ; i< 2 ; i++ ){
      vec2 rid = id + vec2 (i,j)*o;
      vec2 r = p - s*rid;
      dist = min ( dist, sdSphere(vec3(r,torus_pos.z),0.6) );
    }
  }
  
  // dist=sdTorus(torus_pos,vec2(0.6,0.2));
    dist=sdSphere(torus_pos,0.6); //REFLECT_NUM=5;
    REFLECT_NUM=5;

  //dist=sdTorus(torus_pos/0.5,vec2(0.6,0.2))*0.5; //←torus_pos=torus_pos*2.0等とするとバグる
  col=vec3(1.0,0.4,0.4);
  // col=col_rot*col;
  // dist=sdSphere(torus_pos,0.6); REFLECT_NUM=5;

  vec3 box_pos=pos;
  box_pos=box_pos-vec3(-0.55,-0.0,0.2);
  // box_pos=box_pos-vec3(0.55,0.0,-0.2);
  float ndist=sdBox(box_pos,vec3(0.4,0.4,0.4));
  if(ndist<dist){dist=ndist;col=vec3(0.4,0.4,1.0);}
  
  // SHADOW_RATE=1.0;
  // REFLECT_NUM=0;
  // // ↓スムース合体(softmin)
  // float k=3.0+3.0*sin(time*0.4);
  // float ea=exp(-k*dist),eb=exp(-k*ndist);
  // dist=(dist*ea+ndist*eb)/(ea+eb);
  // col=(col*ea+vec3(0.4,0.4,1.0)*eb)/(ea+eb);

  vec3 box2_pos=pos;
  // box2_pos=box2_pos-vec3(-0.6,1.2,0.5+0.4*sin(time*0.5));
  box2_pos=box2_pos-vec3(-0.6,1.2,0.2);
  ndist=sdBox(box2_pos,vec3(0.4,0.1,0.4));
  if(ndist<dist){dist=ndist;col=vec3(0.4,1.0,0.4);}

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
}
vec3 sdfNorm(vec3 p){
  return normalize(vec3(
  sdfColored(p+DX).x-sdfColored(p-DX).x,
  sdfColored(p+DY).x-sdfColored(p-DY).x,
  sdfColored(p+DZ).x-sdfColored(p-DZ).x
  ));
}

float TOTAL_DIST_MAX=10000.0;
bool rayMarch(inout vec3 pos,inout vec3 dir,out vec3 normal,
  in vec3 light_dir,in vec4 ambient, out vec4 color, inout float total_dist){
  vec3 pos0=pos;
  bool hitflag=false;
  // 5. Rayの判定
  float t = 0.0, d;
  for (int i = 0; i < 128; i++ ){ //何回でもok 十分な数
    d = sdfColored(pos).x; //距離関数から現在の距離を求める //回転行列をかける
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


  float l = dot(normal, -light_dir); //法線ベクトルと光源のベクトルの内積（dot)
  l=clamp(l,0.0,1.0);
  vec3 mat_color=sdfColored(pos).yzw;

  // vec3 reflect_ray=(-ray.dir)+2*(normal-(-ray.dir)); //レイが反射していく方向
  vec3 reflect_ray=normalize(2.0*normal*dot(normal,-dir)+dir); //レイが反射していく方向
  vec3 reflect_light=normalize(2.0*normal*dot(normal,-light_dir)+light_dir); //光の反射方向

  float spec_dot=dot(reflect_light,-dir);
  // spec_dot=clamp(spec_dot,0.0,1.0);
  spec_dot=max(spec_dot,0.01);
  float specular=pow(spec_dot,20.0);


  color = vec4(l*mat_color,1.0)+
    vec4(ambient.xyz*mat_color,1.0)+
    vec4(specular,specular,specular,1.0)*0.7;

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
  
  rot=rot_cursor;

  //2. 始点の定義 (カメラの姿勢が定まる
  vec3 camera_pos = vec3(0.0, 0.0, -4.0 + time*1.0); //カメラの位置
  camera_pos = vec3(-1.5, 1.7, -1.4);
  vec3 camera_up = vec3(0.0, 1.0, 0.0);  //カメラの上向きベクトル
  vec3 camera_dir = vec3(0.0, 0.0, 1.0); //カメラの前向きベクトル
  vec3 camera_side = cross(camera_up, camera_dir); //カメラの横向きベクトル (上向きベクトルと前向きベクトルの外積
  camera_up=rot*camera_up;
  camera_dir=rot*camera_dir;
  camera_side=rot*camera_side;


  //8. 光源が当たる方向を定義(z方向に光があたるように
  vec3 light_dir = normalize(vec3(-0.3,-0.6, 1.0)); //マウスを光源とする
  light_dir=normalize(z_axis_rot(time*0.3)*light_dir);
  // light_dir = normalize(rot_cursor*normalize(vec3(-0.0,-0.0, 1.0)));

  // 4. Rayの設定
  Ray ray; //ここはインスタンスか
  ray.pos = camera_pos; //Rayの初期位置
  ray.dir = normalize(pos.x * camera_side + pos.y * camera_up + camera_dir); // Rayの進行方向はカメラの姿勢から求めることができる

  vec3 normal;
  vec4 ambient=vec4(0.2,0.2,0.2,1.0);
  vec4 color;
  float total_dist=0.0;
  bool hitflag=rayMarch(ray.pos,ray.dir,normal,
    light_dir,ambient,color,total_dist);




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

    if(dot(normal,-light_dir)>0.0){
    vec4 tmp_col;
    vec3 tmp_pos=ray.pos,tmp_dir=-light_dir,tmp_norm=normal;
    float tmp_total_dist=total_dist;
      tmp_pos=tmp_pos+normal*MARCHING_DELTA*100.0; //ここを2.0→10.0にしたら直った? 状況次第で100.0
    vec3 tmp_pos_first=tmp_pos;
    bool isInShadow=rayMarch(tmp_pos,tmp_dir,tmp_norm,
    light_dir,ambient,tmp_col,tmp_total_dist);
    if(isInShadow
    // &&sdfColored(ray.pos).yzw!=sdfColored(tmp_pos).yzw
    // ||sdfColored(tmp_pos_first).x<0.0
    )gl_FragColor*=SHADOW_RATE;
    // gl_FragColor*=max(dot(tmp_norm,-light_dir)+1.0,0.0);
    }
    
    //TODO:遠距離なら反射を打ち切ると軽量化?
    for (int i=0; i<REFLECT_NUM; i++) {
      ray.pos=ray.pos+ray.dir*MARCHING_DELTA*100.0; //TODO:ここの2.0調整 状況次第で100.0
      hitflag=rayMarch(ray.pos,ray.dir,normal,
        light_dir,ambient,color,total_dist);
      
      if(hitflag||true)gl_FragColor=gl_FragColor*(0.7+0.3*color);
      else break;

      if(!hitflag)break;
    }

    // vec3 color_hsv=rgb2hsv(gl_FragColor.xyz);
    // gl_FragColor=vec4(hsv2rgb(
    //   vec3(color_hsv.x,color_hsv.y*(1.0-total_dist*1.0/TOTAL_DIST_MAX),color_hsv.z)),1.0);
    // float color_rgbmax=
  }else{ //空色
    gl_FragColor=color;
  }
}
