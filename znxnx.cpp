#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>

#include "clipper2/clipper.h" // Clipper2 include
#include <rapidjson/document.h>

using namespace Clipper2Lib;

constexpr double PI = 3.141592653589793;
constexpr double EARTH_RADIUS = 6378137.0;
constexpr double SCALE = 1e6; // for Clipper

struct LatLon {
    double lat, lon;
};

struct Link {
    int id;
    std::vector<LatLon> coords;
};

// 위경도 -> XY (로컬 평면) 변환
void latlonToXY(double lat, double lon, double refLat, double refLon, double& x, double& y) {
    double dLat = (lat - refLat) * PI / 180.0;
    double dLon = (lon - refLon) * PI / 180.0;
    double refLatRad = refLat * PI / 180.0;

    x = EARTH_RADIUS * dLon * std::cos(refLatRad);
    y = EARTH_RADIUS * dLat;
}

// 사각형 범위 (전방 100m, 좌우 30m)
PathD makeSearchRect(double heading_deg) {
    double theta = heading_deg * PI / 180.0;
    double dx_front = 100.0 * std::cos(theta);
    double dy_front = 100.0 * std::sin(theta);
    double dx_back  = -100.0 * std::cos(theta);
    double dy_back  = -100.0 * std::sin(theta);

    double dx_left  = -30.0 * std::sin(theta);
    double dy_left  =  30.0 * std::cos(theta);
    double dx_right =  30.0 * std::sin(theta);
    double dy_right = -30.0 * std::cos(theta);

    PathD rect(4);
    rect[0] = PointD(dx_back + dx_left, dy_back + dy_left);
    rect[1] = PointD(dx_back + dx_right, dy_back + dy_right);
    rect[2] = PointD(dx_front + dx_right, dy_front + dy_right);
    rect[3] = PointD(dx_front + dx_left, dy_front + dy_left);
    return rect;
}

// GeoJSON 로딩
std::vector<Link> loadLinks(const std::string& path) {
    std::ifstream ifs(path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();

    rapidjson::Document doc;
    doc.Parse(buffer.str().c_str());

    std::vector<Link> links;
    for (auto& feature : doc["features"].GetArray()) {
        Link link;
        link.id = feature["properties"]["id"].GetInt();
        auto& coords = feature["geometry"]["coordinates"];

        for (auto& line : coords.GetArray()) {
            for (auto& point : line.GetArray()) {
                double lon = point[0].GetDouble();
                double lat = point[1].GetDouble();
                link.coords.push_back({ lat, lon });
            }
        }

        links.push_back(link);
    }

    return links;
}

// 자르기 및 결과 반환
std::vector<PathD> clipLinks(const std::vector<Link>& links,
                             double refLat, double refLon,
                             const PathD& searchRect) {
    PathsD result;

    ClipperD clipper;
    clipper.AddClip(searchRect * SCALE, ptSubject, false);

    for (const auto& link : links) {
        PathD path;
        for (auto& ll : link.coords) {
            double x, y;
            latlonToXY(ll.lat, ll.lon, refLat, refLon, x, y);
            path.emplace_back(x, y);
        }

        if (path.size() < 2) continue;

        PathsD clipped;
        clipper.Clear();
        clipper.AddClip(searchRect * SCALE, ptClip);
        clipper.AddSubject(path * SCALE, ptSubject, false);
        clipper.Execute(ClipType::ctIntersection, FillRule::NonZero, clipped);

        for (const auto& p : clipped) {
            result.push_back(p / SCALE); // scale back
        }
    }

    return result;
}

int main() {
    std::string geojsonFile = "links.geojson";
    double gpsLat = 37.5665;
    double gpsLon = 126.9780;
    double heading = 0.0;

    auto links = loadLinks(geojsonFile);
    auto searchRect = makeSearchRect(heading);
    auto clipped = clipLinks(links, gpsLat, gpsLon, searchRect);

    std::cout << "Clipped Link Segments:\n";
    for (const auto& path : clipped) {
        std::cout << "Segment:\n";
        for (const auto& pt : path) {
            std::cout << "  " << pt.x << ", " << pt.y << "\n";
        }
    }

    return 0;
}