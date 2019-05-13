#ifndef MYITEM_H
#define MYITEM_H

#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsItem>

#include <iostream>

using namespace std;

class MyItem : public QGraphicsItem
{
public:
    MyItem();
    QRectF boundingRect() const;
    QRectF bounding_smallRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void advance(int phase);

protected:

private:
    qreal angle;
    qreal speed;
    void DoCollision();
};

class MyBall : public QGraphicsEllipseItem
{
public:
    MyBall();
    QRectF boundingRect() const;
    QPainterPath shape() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // MYITEM_H
