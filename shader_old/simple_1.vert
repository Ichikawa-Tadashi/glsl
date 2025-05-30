#version 120

// simple.vert

void main(void)
{
  vec4 position = gl_ModelViewMatrix * gl_Vertex;
  vec3 normal = normalize(gl_NormalMatrix * gl_Normal);
  vec3 light = normalize((gl_LightSource[0].position * position.w - gl_LightSource[0].position.w * position).xyz);

  float diffuse = max(dot(light, normal), 0.0);
  
  vec3 view = -normalize(position.xyz);
  vec3 halfway = normalize(light + view);
  float specular = pow(max(dot(normal, halfway), 0.0), gl_FrontMaterial.shininess);
//  gl_FrontColor = gl_LightSource[0].diffuse * gl_FrontMaterial.diffuse * diffuse
//                + gl_LightSource[0].specular * gl_FrontMaterial.specular * specular
//                + gl_LightSource[0].ambient * gl_FrontMaterial.ambient;
  gl_FrontColor = gl_FrontLightProduct[0].diffuse * diffuse
                + gl_FrontLightProduct[0].specular * specular
                + gl_FrontLightProduct[0].ambient;

  //gl_FrontColor = vec4(1.0, 0.0, 0.0, 1.0);

  // gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  gl_Position = ftransform();
}
