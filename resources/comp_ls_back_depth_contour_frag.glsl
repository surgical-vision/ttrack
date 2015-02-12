#version 110

uniform sampler2D tex0;

void main(void)
{

  gl_FragColor = texture2D(tex0, gl_TexCoord[0].st);

  //vec2 p = gl_TexCoord[0].st;
  //float f = texture2D(tex0, vec2(p.x, p.y)).x;

  //if (texture2D(tex0, vec2(p.x + 1.0, p.y)).x < f ||
  //  texture2D(tex0, vec2(p.x + 1.0, p.y + 1.0)).x < f ||
  //  texture2D(tex0, vec2(p.x + 1.0, p.y - 1.0)).x < f ||
  //  texture2D(tex0, vec2(p.x - 1.0, p.y + 1.0)).x < f ||
  //  texture2D(tex0, vec2(p.x - 1.0, p.y)).x < f ||
  //  texture2D(tex0, vec2(p.x - 1.0, p.y - 1.0)).x < f ||
  //  texture2D(tex0, vec2(p.x, p.y + 1.0)).x < f ||
  //  texture2D(tex0, vec2(p.x, p.y - 1.0)).x < f)
  //  gl_FragColor = vec4(1.0, 3.0, 2.0, 1.0);
  //else
  //  gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);

}