//
//
//void getHTML()
//{
//textHTML = "";
//textHTML += "<!DOCTYPE html>";
//textHTML += "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />";
//textHTML += "<html lang=\"zh-CN\">";
//textHTML += "<head>";
//textHTML += "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, minimum-scale=0.5, maximum-scale=2.0, user-scalable=yes\" />";
//textHTML += "    <title>Kangal 舵机标定</title>";
//textHTML += "    <style type=\"text/css\">";
//textHTML += "        *{";
//textHTML += "            padding:0;";
//textHTML += "            margin:0;";
//textHTML += "        }";
//textHTML += "        a{";
//textHTML += "          display: block;width: 100%;height: 100%;line-height: 100px;font-size: 14px;text-align: center;text-decoration:none;color:#000000;";
//textHTML += "        }";
//textHTML += "        .content{";
//textHTML += "            width: 200px;height: 200px;position: relative;margin:50px auto;left: 0;top:50%;bottom: 0;right:0;";
//textHTML += "            box-shadow: 0px 0px 550px rgba(255, 255, 255, 0.3) inset,0px 0px 5px #FFFFFF;";
//textHTML += "        }";
//textHTML += "        .quartercircle{";
//textHTML += "            position:absolute;width: 100px;height: 100px;-webkit-border-radius: 100px 0 0 0;";
//textHTML += "        }";
//textHTML += "        .divLeftFront{";
//textHTML += "            top: 0%;left: 0%; transform:rotate(0deg);background-color: bisque ;";
//textHTML += "        }";
//textHTML += "        .divLeftRear{";
//textHTML += "            top: 50%;left: 0%; transform:rotate(-90deg);background-color: burlywood ;";
//textHTML += "        }";
//textHTML += "        .divRightFront{";
//textHTML += "            top: 0%;left: 50%;transform:rotate(90deg);background-color: darkgray ;";
//textHTML += "        }";
//textHTML += "        .divRightRear{";
//textHTML += "            top: 50%;left: 50%;transform:rotate(180deg);background-color: darkkhaki ;";
//textHTML += "        }";
//textHTML += "        .divLeft{";
//textHTML += "            top: 25%;left: -10%; transform:rotate(-45deg);background-color: bisque ;";
//textHTML += "        }";
//textHTML += "        .divLeftLeft{";
//textHTML += "            top: 25%;left: -50%; transform:rotate(-45deg);background-color: bisque ;";
//textHTML += "        }";
//textHTML += "        .divTop{";
//textHTML += "            top: -10%;left: 25%; transform:rotate(45deg);background-color: burlywood ;";
//textHTML += "        }";
//textHTML += "        .divRight{";
//textHTML += "            top: 25%;left: 60%;transform:rotate(135deg);background-color: darkgray ;";
//textHTML += "        }";
//textHTML += "        .divRightRight{";
//textHTML += "            top: 25%;left: 100%;transform:rotate(135deg);background-color: darkgray ;";
//textHTML += "        }";
//textHTML += "        .divBottom{";
//textHTML += "            top: 60%;left: 25%;transform:rotate(-135deg);background-color: darkkhaki ;";
//textHTML += "        }";
//textHTML += "        .circle{";
//textHTML += "          width:50%;height:50%;position: absolute;z-index: 100;top:0%;left:0%;bottom:0;right: 0;margin:auto;border-radius: 100%;background-color: #889900;text-align: center;";
//textHTML += "        }";
//textHTML += "        .circle span{";
//textHTML += "            display: block;width: 100%;height: 100%;line-height: 200px;font-size: 24px;";
//textHTML += "        }";
//textHTML += "        .quartercircle a{";
//textHTML += "            position: absolute;width: 100%;height: 100%;";
//textHTML += "            background: #888888;";
//textHTML += "        }";
//textHTML += "        .quartercircle a:hover{";
//textHTML += "            background: #8BFF7C;";
//textHTML += "        }";
//textHTML += "";
//textHTML += "    </style>";
//textHTML += "";
//textHTML += "<body>";
////textHTML += "<center><h1>Kangal 舵机标定</h1></center>";
//textHTML += "<div class=\"content\" style=\"\">";
//textHTML += "    <form action=\"/\" method=\"get\" accept-charset=\"utf-8\"> ";
//textHTML += "    <div class=\"quartercircle divLeftFront\" style=\"\">";
//textHTML += "      <a href=\"?leg=l1\" style=\"background: no-repeat center;transform:rotate(0deg);\">腿 1</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divLeftRear\" style=\"\">";
//textHTML += "        <a href=\"?leg=l2\" style=\"background: no-repeat center;transform:rotate(90deg);\">腿 2</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divRightFront\" style=\"\">";
//textHTML += "        <a href=\"?leg=l3\" style=\"background: no-repeat center;transform:rotate(-90deg);\">腿 3</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divRightRear\" style=\"\">";
//textHTML += "        <a href=\"?leg=l4\" style=\"display:block;position:absolute;z-index:50;background: no-repeat center;transform:rotate(180deg);\">腿 4</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"circle\" style=\"background-color: red;\"><a href=\"?key=sc\">保存并退出</a></div>";
//textHTML += "  </form>";
//textHTML += "</div>";
//textHTML += "<div class=\"content\" style=\"\">";
//textHTML += "    <form action=\"/\" method=\"get\" accept-charset=\"utf-8\"> ";
//textHTML += "    <div class=\"quartercircle divLeftLeft\" style=\"\">";
//textHTML += "      <a href=\"?shoulder=dec\" style=\"background: no-repeat center;transform:rotate(45deg);\">关节-</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divRightRight\" style=\"\">";
//textHTML += "        <a href=\"?shoulder=inc\" style=\"background: no-repeat center;transform:rotate(-135deg);\">关节+</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divLeft\" style=\"\">";
//textHTML += "      <a href=\"?arm=dec\" style=\"background: no-repeat center;transform:rotate(45deg);\">小腿-</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divTop\" style=\"\">";
//textHTML += "        <a href=\"?ham=inc\" style=\"background: no-repeat center;transform:rotate(-45deg);\">大腿+</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divRight\" style=\"\">";
//textHTML += "        <a href=\"?arm=inc\" style=\"background: no-repeat center;transform:rotate(-135deg);\">小腿+</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divBottom\" style=\"\">";
//textHTML += "        <a href=\"?ham=dec\" style=\"display:block;position:absolute;z-index:50;background: no-repeat center;transform:rotate(135deg);\">大腿-</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"circle\" style=\"background-color: red;\"><a href=\"?key=init\">开始标定</a></div>";
//textHTML += "  </form>";
//textHTML += "</div>";
//textHTML += "<div class=\"content\" style=\"\">";
//textHTML += "    <form action=\"/\" method=\"get\" accept-charset=\"utf-8\"> ";
//textHTML += "    <div class=\"quartercircle divLeft\" style=\"\">";
//textHTML += "      <a href=\"?move=sit\" style=\"background: no-repeat center;transform:rotate(45deg);\">坐下</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divTop\" style=\"\">";
//textHTML += "        <a href=\"?move=inc\" style=\"background: no-repeat center;transform:rotate(-45deg);\">前移+</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divRight\" style=\"\">";
//textHTML += "        <a href=\"?move=stand\" style=\"background: no-repeat center;transform:rotate(-135deg);\">站立</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"quartercircle divBottom\" style=\"\">";
//textHTML += "        <a href=\"?move=dec\" style=\"display:block;position:absolute;z-index:50;background: no-repeat center;transform:rotate(135deg);\">后退-</a>";
//textHTML += "    </div>";
//textHTML += "    <div class=\"circle\" style=\"background-color: red;\"><a href=\"?key=step\">踏步测试</a></div>";
//textHTML += "  </form>";
//textHTML += "</div>";
//
//textHTML += "<br />";
//textHTML += "";
//textHTML += "";
//}
