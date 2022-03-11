$input vWorldPos, vNormal, vTexCoord0, vTangent, vBinormal

#include <forward_pipeline.sh>

// Surface attributes
uniform vec4 uDiffuseColor;

SAMPLER2D(uDiffuseMap, 0);

void main() {
	gl_FragColor = texture2D(uDiffuseMap, vTexCoord0) * uDiffuseColor;
}
