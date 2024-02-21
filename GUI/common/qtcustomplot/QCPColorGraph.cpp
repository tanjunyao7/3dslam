#include "QCPColorGraph.h"
 
QCPColorGraph::QCPColorGraph(QCPAxis *keyAxis, QCPAxis *valueAxis) : QCPGraph(keyAxis, valueAxis) {}
 
QCPColorGraph::~QCPColorGraph(){ }
 
void QCPColorGraph::setData(const QVector<double> & keys, const QVector<double> & values, const QVector<QColor> & colors){
    if (values.size() != colors.size()) return;
    for(int i=0;i<colors_.size();i++)
        if(colors[i]!=colors_[i]){
            printf("tag2\n");
            exit(0);
        }
    colors_ = colors;
    QCPGraph::setData(keys, values);
}
 
void QCPColorGraph::drawScatterPlot(QCPPainter * painter, const QVector<QPointF> & points, const QCPScatterStyle & style) const {
    applyScattersAntialiasingHint(painter);
    int nPoints = points.size();
    if(points.size()!=colors_.size()){
        printf("tag1\n");
        exit(0);
    }
    for (int i = 0; i < nPoints; ++i)
        if (!qIsNaN(points.at(i).x()) && !qIsNaN(points.at(i).y())){
            painter->setPen(colors_[i]);
            style.drawShape(painter, points.at(i));
        }
}