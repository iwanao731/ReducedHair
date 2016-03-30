# A Reduced Model for Interactive Hairs

## 概要
実時間で大量の髪の毛を表現する手法．頭部の様々な動きを入力し，すべての髪の毛をフルシミュレーションした結果を用いてガイドヘアを抽出し，残りのヘアをスキニングによって補間．スキニングウェイトはフルシミュレーションした結果からガイドヘアから最適なウェイトを最適化によって求める．

## 環境
- openFrameworks v0.8.4
- Visual Studio 2012
- Windows 7

## 使用ライブラリ
- Eigen
	- http://eigen.tuxfamily.org/index.php?title=Main_Page
- QuadProg
	- http://quadprog.sourceforge.net/
- boost 1.6.0
	- http://www.boost.org/

## 使用アドオン
- ofxNearestNeighbor
	- https://github.com/neilmendoza/ofxNearestNeighbour
- ofxUI
	- https://github.com/rezaali/ofxUI
- ofxXmlSetting
	- default

## 自作アドオン
- ofxHEMesh
- ofxRay
- ofxMeshUtil



