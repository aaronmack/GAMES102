#version 430 

layout(location=0) in vec3 p;

uniform mat4 model=mat4(1.0);
uniform mat4 view=mat4(1.0);
uniform mat4 projection=mat4(1.0);
uniform float time;
out vec3 pos;
void main(){
   pos=p;
   gl_Position=projection*view*model*vec4(p,1.0);
  // gl_PointSize=gl_VertexID*5; 
}
