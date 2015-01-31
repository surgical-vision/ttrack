#version 110

uniform sampler2D tex0;

void main(void)
{

  gl_FragColor = texture2D(tex0, gl_TexCoord[0].st);

}