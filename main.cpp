/*
Даны n точек в пространстве. Никакие 4 точки не лежат в одной плоскости. 
Найдите выпуклую оболочку этих точек.

Входные данные

Первая строка содержит число m — количество тестов. 
В последующих строках описаны сами тесты. 
Каждый тест начинается со строки, содержащей n (1 ≤ n ≤ 1000) — число точек. 
Далее, в n строках даны по три числа — координаты точек. 
Все координаты целые, не превосходят по модулю 500. 
Общее количество точек не превосходит 2100.

Выходные данные

Для каждого теста выведите следующее. 
В первую строку выведите количество граней m. 
Далее в последующие m строк выведите описание граней: количество вершин и 
номера точек в исходном множестве. Точки нумеруются в том порядке, в котором 
они даны во входном файле. Точки в пределах грани должны быть отсортированы в 
порядке против часовой стрелки относительно внешней нормали к грани.
 */

#include <algorithm>
#include <iostream>
#include <vector>

struct Point {
  double x;
  double y;
  double z;
};

using Vector = Point;

Point operator*(double lhs, const Point& rhs) {
  return {lhs*rhs.x, lhs*rhs.y, lhs*rhs.z};
}

Point operator+(const Point& lhs, const Point& rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Point operator-(const Point& lhs, const Point& rhs) {
  return lhs + (-1)*rhs;
}

struct Face {
  int point1_id;
  int point2_id;
  int point3_id;
};

bool operator<(const Face& lhs, const Face& rhs) {
  if (lhs.point1_id < rhs.point1_id) {
    return true;
  }
  if (lhs.point1_id == rhs.point1_id &&
      lhs.point2_id < rhs.point2_id) {
    return true;
  }
  if (lhs.point1_id == rhs.point1_id &&
      lhs.point2_id == rhs.point2_id &&
      lhs.point3_id < rhs.point3_id) {
    return true;
  }
  return false;
}

static bool CompareFacesByPointOrder(const Face& lhs, const Face& rhs) {
  if (lhs.point1_id < rhs.point1_id) {
    return true;
  }
  if (lhs.point1_id == rhs.point1_id &&
      lhs.point2_id < rhs.point2_id) {
    return true;
  }
  if (lhs.point1_id == rhs.point1_id &&
      lhs.point2_id == rhs.point2_id &&
      lhs.point3_id < rhs.point3_id) {
    return true;
  }
  return false;
}

std::vector<Face> BuildConvexHull(const std::vector<Point>& points);

int main() {
  int M;
  std::cin >> M;
  for (int i = 0; i < M; ++i) {
    int N;
    std::cin >> N;
    std::vector<Point> points(N);
    for (int j = 0; j < N; ++j) {
      std::cin >> points[j].x >> points[j].y >> points[j].z;
    }
    auto convex_hull = BuildConvexHull(points);
    std::cout << convex_hull.size() << std::endl;
    std::sort(convex_hull.begin(), convex_hull.end(), CompareFacesByPointOrder);
    for (auto face : convex_hull) {
      std::cout << 3 << ' ' << face.point1_id << ' ' << face.point2_id << ' ' <<
                face.point3_id << std::endl;
    }
  }
  return 0;
}

double DotProduct(const Vector& pt1, const Vector& pt2) {
  return pt1.x*pt2.x + pt1.y*pt2.y + pt1.z*pt2.z;
}

Vector CrossProduct(const Vector& vec1, const Vector& vec2) {
  double x = vec1.y*vec2.z - vec1.z*vec2.y;
  double y = vec1.z*vec2.x - vec1.x*vec2.z;
  double z = vec1.x*vec2.y - vec1.y*vec2.x;
  return {x, y, z};
}

std::vector<Face> BuildTetrahedron(const std::vector<Point>& points) {
  Face base = {0, 1, 2};
  Vector vec1 = points[1] - points[0];
  Vector vec2 = points[2] - points[0];
  Vector norm_vec = CrossProduct(vec1, vec2);
  Vector vec3 = points[3] - points[0];
  if (DotProduct(norm_vec, vec3) > 0.0) {
    std::swap(base.point2_id, base.point3_id);
  }
  Face face1 = {base.point1_id, 3, base.point2_id};
  Face face2 = {base.point1_id, base.point3_id, 3};
  if (base.point2_id < base.point3_id) {
    return {base, face1, face2, {base.point2_id, 3, base.point3_id}};
  } else {
    return {base, face1, face2, {base.point3_id, base.point2_id, 3}};
  }
}

bool FaceVisibleFromPoint(const Face& face,
                          const std::vector<Point>& points,
                          const Point& point) {
  Vector vec1 = points[face.point2_id] - points[face.point1_id];
  Vector vec2 = points[face.point3_id] - points[face.point1_id];
  Vector norm_vec = CrossProduct(vec1, vec2);
  Vector vec3 = point - points[face.point1_id];
  return DotProduct(vec3, norm_vec) > 0;
}

std::vector<Face> FindInvisibleEdges(const std::vector<Face>& convex_hull,
                                     const std::vector<Point>& points,
                                     int cur_point_id,
                                     std::vector<std::vector<bool>>& dead_edges) {
  std::vector<Face> invisible_faces;
  for (auto face : convex_hull) {
    if (FaceVisibleFromPoint(face, points, points[cur_point_id])) {
      dead_edges[face.point1_id][face.point2_id] =
      dead_edges[face.point2_id][face.point3_id] =
      dead_edges[face.point3_id][face.point1_id] = true;
    } else {
      invisible_faces.push_back(face);
    }
  }
  return invisible_faces;
}

void AddNewFaces(std::vector<Face>& convex_hull,
                 std::vector<Face>& invisible_faces,
                 int cur_pt_id,
                 std::vector<std::vector<bool>>& dead_edges) {
  convex_hull.clear();
  for (auto face : invisible_faces) {
    int face_points[] = {face.point1_id, face.point2_id, face.point3_id};
    for (int j = 0; j < 3; ++j) {
      int pt1 = face_points[j];
      int pt2 = face_points[(j + 1)%3];
      if (dead_edges[pt2][pt1]) {
        if (pt1 > pt2) {
          convex_hull.push_back({pt2, pt1, cur_pt_id});
        } else {
          convex_hull.push_back({pt1, cur_pt_id, pt2});
        }
        dead_edges[pt2][pt1] = false;
      }
    }
  }
  convex_hull.insert(convex_hull.end(),
                     invisible_faces.begin(),
                     invisible_faces.end());
}

std::vector<Face> BuildConvexHull(const std::vector<Point>& points) {
  auto convex_hull = BuildTetrahedron(points);
  std::vector<std::vector<bool>> dead_edges(points.size(),
                                            std::vector<bool>(points.size(),
                                                              false));
  for (int i = 4; i < points.size(); ++i) {
    std::vector<Face> invisible_faces =
        FindInvisibleEdges(convex_hull, points, i, dead_edges);
    AddNewFaces(convex_hull, invisible_faces, i, dead_edges);
  }
  return convex_hull;
}
