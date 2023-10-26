#ifndef __POINT_SEARCHER_H_
#define __POINT_SEARCHER_H_

#include <algorithm>
#include <boost/multi_array.hpp>
#include <iostream>
#include <vector>

#include "kdtree.h"


namespace algo {

/** \brief Nearest searching tools in 2D points based on KD-Tree
        \code
        PointList pts;
        for(int i=0;i<100;i++){
                pts.push_back(Point(i,100-i));
        }
        PointSearcher searcher;
        searcher.buildTree(pts); //build kd tree for input points
        Point query(1,1);
        Point result = searcher.nearest(query); //find nearst point to query
        \endcode
*/

template<typename Point2DT> class Point2DSearcher
{
public:
    Point2DSearcher()
        : m_kdtree(nullptr)
    {}
    Point2DSearcher(const std::vector<Point2DT>& points)
        : m_kdtree(nullptr)
    {
        buildTree(points);
    }

    ~Point2DSearcher()
    {
        if (m_kdtree) delete m_kdtree;
        m_kdtree = nullptr;
    }

    void buildTree(const std::vector<Point2DT>& points)
    {
        if (m_kdtree) delete m_kdtree;

        m_origin_data = uniquePoints(points);
        if (m_origin_data.empty()) {
            std::cout << "Warn: PointSearcher, build kdtree faild, points is empty." << std::endl;
            return;
        }

        int point_cnt = m_origin_data.size();
        m_data.resize(boost::extents[point_cnt][2]);
        for (int i = 0; i < point_cnt; ++i) {
            m_data[i][0] = m_origin_data[i].x;
            m_data[i][1] = m_origin_data[i].y;
        }

        m_kdtree = new KDTree(m_data);
    }

    Point2DT nearest(const ::algo::Point& p)
    {
        std::vector<Point2DT> points = KSearch(p, 1);
        if (points.empty()) {
            std::cout << "ERROR: PointSearcher, nearest search failed." << std::endl;
            return Point2DT();
        }

        return points[0];
    }
    std::vector<Point2DT> KSearch(const ::algo::Point& p, int n)
    {
        KDTreeResultVector    neighbor = KSearchIdx(p, n);
        std::vector<Point2DT> res(neighbor.size());
        for (int i = 0; i < neighbor.size(); ++i) {
            res[i] = m_origin_data[neighbor[i].idx];
        }
        return res;
    }
    std::vector<Point2DT> RSearch(const ::algo::Point& p, double r)
    {
        KDTreeResultVector    neighbor = RSearchIdx(p, r);
        std::vector<Point2DT> res(neighbor.size());
        for (int i = 0; i < neighbor.size(); ++i) {
            res[i] = m_origin_data[neighbor[i].idx];
        }
        return res;
    }

private:
    KDTreeResultVector KSearchIdx(const ::algo::Point& p, int n)
    {
        KDTreeResultVector neighbor;
        if (m_kdtree) {
            std::vector<float> query(2);
            query[0] = p.x;
            query[1] = p.y;
            m_kdtree->n_nearest(query, n, neighbor);
        }
        return neighbor;
    }
    KDTreeResultVector RSearchIdx(const ::algo::Point& p, double r)
    {
        KDTreeResultVector neighbor;
        if (m_kdtree) {
            std::vector<float> query(2);
            query[0] = p.x;
            query[1] = p.y;
            m_kdtree->r_nearest(query, r * r,
                                neighbor);   // r_nearest搜索的是(square Euclidean distance)小于r的值
        }
        return neighbor;
    }

private:
    std::vector<Point2DT> uniquePoints(const std::vector<Point2DT>& points)
    {
        std::vector<Point2DT> res = points;
        std::sort(res.begin(), res.end());
        res.erase(std::unique(res.begin(), res.end()), res.end());
        return res;
    }

private:
    KDTree*               m_kdtree;
    KDTreeArray           m_data;
    std::vector<Point2DT> m_origin_data;
};

// typedef Point2DSearcher<::algo::Point> PointSearcher;

}   // namespace algo

#endif
