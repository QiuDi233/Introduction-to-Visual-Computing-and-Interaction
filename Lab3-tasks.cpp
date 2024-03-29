#include "Labs/3-Rendering/tasks.h"

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    float dotProduct(glm::vec3 v1, glm::vec3 v2) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }
    glm::vec3 crossProduct(glm::vec3 v1, glm::vec3 v2) {
        glm::vec3 result(0);
        result.x = v1.y * v2.z - v2.y * v1.z;
        result.y = v2.x * v1.z - v2.z * v1.x;
        result.z = v1.x * v2.y - v2.x * v1.y;
        return result;
    }
    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        auto   O = ray.Origin, D = ray.Direction, P0 = p1, P1 = p2, P2 = p3;
        auto   E1 = P1 - P0, E2 = P2 - P0, S = O - P0;
        auto   S1 = crossProduct(D, E2), S2 = crossProduct(S, E1);
        float  tnear = dotProduct(S2, E2) / dotProduct(S1, E1);
        float  u     = dotProduct(S1, S) / dotProduct(S1, E1);
        float  v     = dotProduct(S2, D) / dotProduct(S1, E1);
        double eps   = -1e-6;
        if (tnear > eps && u > eps && v > eps && 1 - u - v > eps) {
            output.t = tnear;
            output.u = u;
            output.v = v;
            return true;
        }
        return false;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);
        float     ambient = 0.05f;

        for (int depth = 0; depth < maxDepth; depth++) {
            glm::vec3 pos, n, kd, ks;
            float     alpha, shininess;
            auto      rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            pos       = rayHit.IntersectPosition;
            n         = rayHit.IntersectNormal;
            kd        = rayHit.IntersectAlbedo;
            ks        = rayHit.IntersectMetaSpec;
            alpha     = rayHit.IntersectAlbedo.w;
            shininess = rayHit.IntersectMetaSpec.w * 256;
            if (alpha < .2)
                ray.Origin = pos;
            while (alpha < .2)
            {
                auto rayHit = intersector.IntersectRay(ray);
                if (! rayHit.IntersectState) return color;
                pos       = rayHit.IntersectPosition;
                n         = rayHit.IntersectNormal;
                kd        = rayHit.IntersectAlbedo;
                ks        = rayHit.IntersectMetaSpec;
                alpha     = rayHit.IntersectAlbedo.w;
                shininess = rayHit.IntersectMetaSpec.w * 256;
                if (alpha < .2)
                    ray.Origin = pos;
            } 

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here
            result = kd * ambient;

            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        auto shadowRayHit = intersector.IntersectRay(Ray(pos, glm::normalize(l)));
                        while (shadowRayHit.IntersectState && shadowRayHit.IntersectAlbedo.w < 0.2)
                            shadowRayHit = intersector.IntersectRay(Ray(shadowRayHit.IntersectPosition, glm::normalize(l)));
                        if (shadowRayHit.IntersectState) {
                            glm::vec3 tmp  = shadowRayHit.IntersectPosition - pos;
                            if (glm::dot(tmp, tmp) < glm::dot(l, l))
                                attenuation = 0.0f;
                        }
                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        auto shadowRayHit = intersector.IntersectRay(Ray(pos, glm::normalize(l)));
                        while (shadowRayHit.IntersectState && shadowRayHit.IntersectAlbedo.w < 0.2)
                            shadowRayHit = intersector.IntersectRay(Ray(shadowRayHit.IntersectPosition, glm::normalize(l)));
                        if (shadowRayHit.IntersectState)
                            attenuation = 0.0f;
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                glm::vec3 h         = glm::normalize(-ray.Direction + glm::normalize(l));
                float     spec = glm::pow(glm::max(glm::dot(h, n), 0.0f), shininess);
                float     diff = glm::max(glm::dot(glm::normalize(l), n), 0.0f);
                result += light.Intensity * attenuation * (diff * kd + spec * ks);
            }
           

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering