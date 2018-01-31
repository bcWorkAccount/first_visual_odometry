1、计算z（这里我为了利用keypoint，随便找了几张图片，利用opencv提取特征点，匹配的）
  提取左右关键点，利用左右关键点之差计算视差，然后利用相机模型计算出x，y，z

   unsigned int ulr=leftKPs[i].x-rightKPs[i].x;
            if(ulr==0) continue;
           point3D[2]=fx*d/ulr;
           point3D[0]=(leftKPs[i].x-cx)*point3D[2]/fx;
           point3D[1]=(leftKPs[i].y-cy)*point3D[2]/fy;
           pointcloud.push_back(point3D);
            }




2、BA计算位姿

利用计算出来的三维点和后一帧的关键点，做BA
 Vector3d p_camera= T_esti * pointcloud[i];//相機坐標系（x、y、z）
           Vector3d uvs = K*p_camera;//投影模型
           Vector3d uv = K*p_camera/p_camera(2);//歸一化

    定义误差

           Vector2d e;
           e={left2KPs[i].x-uv(0),left2KPs[i].y-uv(1)};


利用高斯牛顿做最小二乘解最佳位姿

 Matrix<double, 2, 6> J;

            double X=p_camera(0);
            double Y=p_camera(1);
            double Z=p_camera(2);
            double Z2=p_camera(2)*p_camera(2);

            J<<-fx/Z,   0,  fx*X/Z2, fx*X*Y/Z2, -fx-fx*X*X/Z2,  fx*Y/Z,
                0, -fy/Z, fy*Y/Z2, fy+fy*Y*Y/Z2, -fy*X*Y/Z2,  -fy*X/Z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx 
        Vector6d dx;
        dx=H.ldlt().solve(b);

更新得到的最佳李群

 T_esti=Sophus::SE3::exp(dx)*T_esti;

