1������z��������Ϊ������keypoint��������˼���ͼƬ������opencv��ȡ�����㣬ƥ��ģ�
  ��ȡ���ҹؼ��㣬�������ҹؼ���֮������ӲȻ���������ģ�ͼ����x��y��z

   unsigned int ulr=leftKPs[i].x-rightKPs[i].x;
            if(ulr==0) continue;
           point3D[2]=fx*d/ulr;
           point3D[0]=(leftKPs[i].x-cx)*point3D[2]/fx;
           point3D[1]=(leftKPs[i].y-cy)*point3D[2]/fy;
           pointcloud.push_back(point3D);
            }




2��BA����λ��

���ü����������ά��ͺ�һ֡�Ĺؼ��㣬��BA
 Vector3d p_camera= T_esti * pointcloud[i];//���C����ϵ��x��y��z��
           Vector3d uvs = K*p_camera;//ͶӰģ��
           Vector3d uv = K*p_camera/p_camera(2);//�wһ��

    �������

           Vector2d e;
           e={left2KPs[i].x-uv(0),left2KPs[i].y-uv(1)};


���ø�˹ţ������С���˽����λ��

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

���µõ��������Ⱥ

 T_esti=Sophus::SE3::exp(dx)*T_esti;

