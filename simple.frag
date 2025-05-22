#version 120

// simple.frag

varying vec4 position;
varying vec3 normal;

uniform vec2 resolution;

float sdSphere( vec3 p/*, float s */)
{
  //return length(p)-s;
  return length(p)-2.0;
}

float NORM_DELTA=0.01;
vec3 DX=vec3(NORM_DELTA,0.0,0.0);
vec3 DY=vec3(0.0,NORM_DELTA,0.0);
vec3 DZ=vec3(0.0,0.0,NORM_DELTA);
vec3 sdSphere_norm(vec3 p){
  return normalize(vec3(
  sdSphere(p+DX)-sdSphere(p-DX),
  sdSphere(p+DY)-sdSphere(p-DY),
  sdSphere(p+DZ)-sdSphere(p-DZ)
  ));
}

void main (void)
{
  /*
  vec3 light = normalize((gl_LightSource[0].position * position.w - gl_LightSource[0].position.w * position).xyz);
  vec3 fnormal = normalize(normal);
  float diffuse = max(dot(light, fnormal), 0.0);
  
  vec3 view = -normalize(position.xyz);
  vec3 halfway = normalize(light + view);
  float specular = pow(max(dot(fnormal, halfway), 0.0), gl_FrontMaterial.shininess);
  gl_FragColor = gl_FrontLightProduct[0].diffuse * diffuse
                + gl_FrontLightProduct[0].specular * specular
                + gl_FrontLightProduct[0].ambient;

  //gl_FragColor.x=(gl_FragColor.x>0.4?0.8:0.2);
  //gl_FragColor.y=(gl_FragColor.y>0.4?0.8:0.2);
  //gl_FragColor.z=(gl_FragColor.z>0.4?0.8:0.2);
  gl_FragColor=floor(gl_FragColor*4-0.3)*0.3+0.2;

  //vec3 edge=vec3(0.5,0.5,0.5);
  //gl_FragColor=step(edge,gl_FragColor)*0.6+0.2;
  gl_FragColor=vec4(1.0,1.0,1.0,1.0);

  vec2 p = (gl_FragCoord.xy * 2.0 - resolution) / min(resolution.x, resolution.y); // rはresolution -1~1にnorm
  //gl_FragColor.xy=gl_FragCoord.xy/200.0;
  gl_FragColor.xy=(p+1.0)/2.0;
  */

  


  vec2 pos = ( gl_FragCoord.xy * 2.0 - resolution) / max(resolution.x, resolution.y);
  gl_FragColor = vec4(pos, 0.0, 1.0); //最終的な色の描画
}
