#version 430 
out vec4 FragColor;
in vec3 pos;
uniform  vec3 color=vec3(1.0,0.5,0.0);
void main() {	
      vec3 sun=vec3(0,1,0);
      vec3 n= normalize(cross(dFdx(pos),dFdy(pos)));
      float luma=dot(sun,n)*0.5+0.5;
      if(color==vec3(1.0,0.0,0.0)){
         FragColor= vec4(color,1.0);  
      }else{
        FragColor= vec4(color*luma,1.0); 
      }
}