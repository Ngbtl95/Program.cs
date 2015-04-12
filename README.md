using System;
using System.Linq;
using AIRLab.Mathematics;
using ClientBase;
using CommonTypes;
using CVARC.Basic.Controllers;
using CVARC.Network;
using RepairTheStarship.Sensors;
using MapHelper;
using System.Collections.Generic;
using CVARC.Basic;

namespace Example
{
    internal class Program
    {
        private static readonly ClientSettings Settings = new ClientSettings
        {
            Side = Side.Left, //Переключив это поле, можно отладить алгоритм для левой или правой стороны, а также для произвольной стороны, назначенной сервером
            LevelName = LevelName.Level2, //Задается уровень, в котором вы хотите принять участие
            MapNumber = -1 //Задавая различные значения этого поля, вы можете сгенерировать различные случайные карты
        };

        private static void Main(string[] args)
        {
            var server = new CvarcClient(args, Settings).GetServer<PositionSensorsData>();
            var helloPackageAns = server.Run();
            var side = helloPackageAns.RealSide;
            var sensorsData = helloPackageAns.SensorsData;
            string lastColorDetail = "";

            while (sensorsData.MapSensor.MapItems.GetDatails().FirstOrDefault() != null)
            {
                if (!sensorsData.DetailsInfo.HasGrippedDetail)
                {
                    var x = sensorsData.Position.PositionsData[sensorsData.RobotId.Id].X;
                    var y = sensorsData.Position.PositionsData[sensorsData.RobotId.Id].Y;
                    var min = sensorsData.MapSensor.
                                            MapItems.
                                            GetDatails().
                                            Min(item => new Tuple<double, string>(item.Item1.Distance(x, y), item.Item2));
                    if (min.Item1 < 25)
                    {
                        lastColorDetail = min.Item2;
                        sensorsData = server.SendCommand(new Command { Action = CommandAction.Grip, Time = 1 });
                    }
                    if (!sensorsData.DetailsInfo.HasGrippedDetail)
                        sensorsData = server.SendCommand(GetStepToDetail(sensorsData));
                }
                else
                {
                    var x = sensorsData.Position.PositionsData[sensorsData.RobotId.Id].X;
                    var y = sensorsData.Position.PositionsData[sensorsData.RobotId.Id].Y;
                    var min = sensorsData.MapSensor.
                                            MapItems.
                                            GetSockets(lastColorDetail).
                                            Select(item => item.Item1).
                                            Min(item => item.Distance(x, y));
                    if (min < 35)
                    {
                        sensorsData = server.SendCommand(new Command { Action = CommandAction.Release, Time = 1 });
                    }
                    if (sensorsData.DetailsInfo.HasGrippedDetail)
                        sensorsData = server.SendCommand(GetStepToWall(sensorsData, lastColorDetail));
                }
            }

            server.Exit();
        }

        private static Command GetStepToWall(PositionSensorsData data, string color)
        {
            var disabledEgde = GetDisabledEdge(data.MapSensor.MapItems, 4, 6);
            var map = new int[4, 6];
            var robot = data.Position.PositionsData[data.RobotId.Id];
            var robot2 = data.Position.PositionsData[(data.RobotId.Id + 1) % 2];
            var start = Normalize(robot.X, robot.Y); 
            map[start.Y, start.X] = 1;
            map[NormalizeY(robot2.Y), NormalizeX(robot2.X)] = -1;
            var queue = new Queue<Point>();
            queue.Enqueue(start);
            while (queue.Count!=0)
            {
                var cur = queue.Dequeue();
                foreach (var p in GetNeighbouringCells(cur))
                    if (p.X >= 0 && p.Y >= 0 && p.X < map.GetLength(1) && p.Y < map.GetLength(0) && map[p.Y, p.X] == 0 && (!disabledEgde[cur.Y, cur.X, p.Y, p.X]))
                    {
                        map[p.Y, p.X] = map[cur.Y, cur.X] + 1;
                        queue.Enqueue(p);
                    }
            }
            var f = data.MapSensor.
                                MapItems.
                                GetSockets(color).
                                SelectMany(item => GetNeighbouringCells(item.Item1, item.Item2)).
                                Min(item =>
                                {
                                    if (map[item.Y, item.X] != 0 && map[item.Y, item.X] != -1)
                                        return new Tuple<int, Point>(map[item.Y, item.X], item);
                                    else
                                        return new Tuple<int, Point>(int.MaxValue, item);
                                }).
                                Item2;
            Point next;
            if (f.X == start.X && f.Y == start.Y)
            {
                var x = data.Position.PositionsData[data.RobotId.Id].X;
                var y = data.Position.PositionsData[data.RobotId.Id].Y;
                var finish = data.MapSensor.
                                    MapItems.
                                    GetSockets(color).
                                    Min(item => new Tuple<double, Point2D>(Normalize(item.Item1).Distance(new Point((int)x, (int)y)), item.Item1)).
                                    Item2;
                next = new Point((int)finish.X, (int)finish.Y);
            }
            else
            {
                var n = f;
                while (f.X != start.X || f.Y != start.Y)
                {
                    n = f;
                    foreach (var p in GetNeighbouringCells(f))
                        if (p.X >= 0 && p.Y >= 0 && p.X < map.GetLength(1) && p.Y <= map.GetLength(0) && (map[p.Y, p.X] == map[f.Y, f.X] - 1) && (!disabledEgde[f.Y, f.X, p.Y, p.X]))
                        {
                            f = p;
                            break;
                        }
                }
                next = new Point(n.X * 50 + 25 - 150, 100 - (n.Y * 50 + 25));
            }

            return MotionDetection(robot, next);
        } 
        
        public static Command MotionDetection(CVARC.Basic.Sensors.PositionData robot, Point next)
        {
            var e = Angle.FromRad(Math.Atan2(next.Y - robot.Y, next.X - robot.X)).Grad;
            if (((int)e + 360) % 360 == ((int)robot.Angle + 360) % 360)
                return new Command { LinearVelocity = 50, Time = Math.Abs(next.Distance(new Point((int)robot.X, (int)robot.Y)) - 10) / 50.0 };
            else
                if ((Math.Abs((int)e - (int)robot.Angle) < 180))
                    return new Command { AngularVelocity = Angle.FromGrad(Math.Sign(e - robot.Angle) * 90), Time = Math.Abs(e - robot.Angle) / 90.0 };
                else
                    if (Math.Abs((int)e - (int)robot.Angle) < 200)
                        return new Command { LinearVelocity = -50, Time = Math.Abs(next.Distance(new Point((int)robot.X, (int)robot.Y))) / 50.0 };
                    else
                        return new Command { AngularVelocity = Angle.FromGrad(Math.Sign(robot.Angle - e) * 90), Time = Math.Abs(360 + robot.Angle - e) / 90.0 };
        }

        public static Command GetStepToDetail(PositionSensorsData data)
        {
            var disabledEgde = GetDisabledEdge(data.MapSensor.MapItems, 4, 6);
            var map = new int[4, 6];
            var robot = data.Position.PositionsData[data.RobotId.Id];
            var robot2 = data.Position.PositionsData[(data.RobotId.Id + 1) % 2];
            var start = Normalize(robot.X, robot.Y);
            map[start.Y, start.X] = 1;
            map[NormalizeY(robot2.Y), NormalizeX(robot2.X)] = -1;
            var queue = new Queue<Point>();
            queue.Enqueue(start);
            while (queue.Count != 0)
            {
                var cur = queue.Dequeue();
                foreach (var p in GetNeighbouringCells(cur))
                    if (p.X >= 0 && p.Y >= 0 && p.X < map.GetLength(1) && p.Y < map.GetLength(0) && map[p.Y, p.X] == 0 && (!disabledEgde[cur.Y, cur.X, p.Y, p.X]))
                    {
                        map[p.Y, p.X] = map[cur.Y, cur.X] + 1;
                        queue.Enqueue(p);
                    }
            }
            var f = data.MapSensor.
                        MapItems.
                        GetDatails().
                        Select(item => Normalize(item.Item1)).
                        Min(item =>
                        {
                            if (map[item.Y, item.X] != 0 && map[item.Y, item.X] != -1)
                                return new Tuple<int, Point>(map[item.Y, item.X], item);
                            else
                                return new Tuple<int, Point>(int.MaxValue, item);
                        }).
                        Item2;
            if (map[f.Y, f.X] == 0)
                throw new Exception();
            Point next;
            if (f.X == start.X && f.Y == start.Y)
            {
                var x = data.Position.PositionsData[data.RobotId.Id].X;
                var y = data.Position.PositionsData[data.RobotId.Id].Y;
                var finish = data.MapSensor.
                                    MapItems.
                                    GetDatails().
                                    Min(item => new Tuple<double, Point2D>(Normalize(item.Item1).Distance(new Point((int)x, (int)y)), item.Item1)).
                                    Item2;
                next = new Point((int)finish.X, (int)finish.Y);
            }
            else
            {
                var n = f;
                while (f.X != start.X || f.Y != start.Y)
                {
                    n = f;
                    foreach (var p in GetNeighbouringCells(f))
                        if (p.X >= 0 && p.Y >= 0 && p.X < map.GetLength(1) && p.Y <= map.GetLength(0) && (map[p.Y, p.X] == map[f.Y, f.X] - 1) && (!disabledEgde[f.Y, f.X, p.Y, p.X]))
                        {
                            f = p;
                            break;
                        }
                }
                next = new Point(n.X * 50 + 25 - 150, 100 - (n.Y * 50 + 25));
            }
            return MotionDetection(robot, next);
        }

        static IEnumerable<Point> GetNeighbouringCells(Point2D point, bool type)
        {
            var p = Normalize(point);
            if (type)
            {
                if (p.X - 1 >= 0)
                    yield return new Point(p.X - 1, p.Y);
            }
            else if (p.Y - 1 >= 0)
                yield return new Point(p.X, p.Y - 1);
            yield return new Point(p.X, p.Y);
        }

        static IEnumerable<Point> GetNeighbouringCells(Point point)
        {
            return
                from x in new[] { -1, 0, 1 }
                from y in new[] { -1, 0, 1 }
                where Math.Abs(x) + Math.Abs(y) == 1
                let p = new Point(point.X + x, point.Y + y)
                select p;
        }

        static bool[, , ,] GetDisabledEdge(MapItem[] items, int w, int h)
        {
            var ans = new bool[w, h, w, h];
            foreach (var point in items.GetVerticalWalls())
            {
                var p = Normalize(point);
                ans[p.Y, p.X, p.Y, p.X - 1] = true;
                ans[p.Y, p.X - 1, p.Y, p.X] = true;
            }
            foreach (var point in items.GetVerticalSockets())
            {
                var p = Normalize(point);
                ans[p.Y, p.X, p.Y, p.X - 1] = true;
                ans[p.Y, p.X - 1, p.Y, p.X] = true;
            }
            foreach (var point in items.GetHorizontalWalls())
            {
                var p = Normalize(point);
                ans[p.Y, p.X, p.Y - 1, p.X] = true;
                ans[p.Y - 1, p.X, p.Y, p.X] = true;
            }
            foreach (var point in items.GetHorizontalSockets())
            {
                var p = Normalize(point);
                ans[p.Y, p.X, p.Y - 1, p.X] = true;
                ans[p.Y - 1, p.X, p.Y, p.X] = true;
            }
            return ans;
        }

        static Point Normalize(Point2D point)
        {
            return new Point(NormalizeX(point.X), NormalizeY(point.Y));
        }

        static Point Normalize(double x, double y)
        {
            return new Point(NormalizeX(x), NormalizeY(y));
        }

        static int NormalizeX(double x)
        {
            return ((int)x + 150) / 50;
        }

        static int NormalizeY(double y)
        {
            return (100 - (int)y) / 50;
        }
    }

    public static class ForPoint2D
    {
        public static double Distance(this Point2D a, double x, double y)
        {
            return Math.Sqrt((a.X - x) * (a.X - x) + (a.Y - y) * (a.Y - y));
        }
    }

    public static class ForPoint
    {
        public static double Distance(this Point a, Point b)
        {
            return Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
        }
    }

    public static class ForMapItems
    {
        public static IEnumerable<Tuple<Point2D, string>> GetDatails(this MapItem[] items)
        {
            return items.Where(item => item.Tag.EndsWith("Detail")).Select(item => new Tuple<Point2D, string>(new Point2D(item.X, item.Y), item.Tag.Substring(0, item.Tag.Length - 6)));
        }

        public static IEnumerable<Point2D> GetVerticalSockets(this MapItem[] items)
        {
            return items.Where(item => item.Tag.EndsWith("Socket")).Where(item => item.Tag.StartsWith("Vertical")).Select(item => new Point2D(item.X, item.Y));
        }

        public static IEnumerable<Point2D> GetHorizontalSockets(this MapItem[] items)
        {
            return items.Where(item => item.Tag.EndsWith("Socket")).Where(item => item.Tag.StartsWith("Horizontal")).Select(item => new Point2D(item.X, item.Y));
        }

        public static IEnumerable<Tuple<Point2D, bool>> GetSockets(this MapItem[] items, string color)
        {
            return items.Where(item => item.Tag.EndsWith(color + "Socket")).Select(item => new Tuple<Point2D, bool>(new Point2D(item.X, item.Y), item.Tag.IndexOf("Vertical") != -1));
        }

        public static IEnumerable<Point2D> GetVerticalWalls(this MapItem[] items)
        {
            return items.Where(item => item.Tag.EndsWith("Wall")).Where(item => item.Tag.StartsWith("Vertical")).Select(item => new Point2D(item.X, item.Y));
        }

        public static IEnumerable<Point2D> GetHorizontalWalls(this MapItem[] items)
        {
            return items.Where(item => item.Tag.EndsWith("Wall")).Where(item => item.Tag.StartsWith("Horizontal")).Select(item => new Point2D(item.X, item.Y));
        }
    }
}
