# Guide d'Intégration de ROS 2 Iron avec MySQL pour l'Enregistrement des Messages

## Configuration de la Base de Données

### Installation de MySQL

```bash
sudo apt update
sudo apt install mysql-server
sudo mysql_secure_installation
```

Durant cette étape, il vous sera demandé de configurer la sécurité de votre installation MySQL, y compris la définition d'un mot de passe root, la désactivation de la connexion root à distance, la suppression des bases de données de test et des utilisateurs anonymes, et le rechargement des tables de privilèges.

### Création de l'Utilisateur et de la Base de Données

Connectez-vous à MySQL avec l'utilisateur root :

```bash
sudo mysql -u root -p
```

Créez un nouvel utilisateur et une base de données pour votre application :

```sql
CREATE USER 'ros2'@'localhost' IDENTIFIED BY 'password';
CREATE DATABASE rosdb;
GRANT ALL PRIVILEGES ON rosdb.* TO 'ros2'@'localhost';
FLUSH PRIVILEGES;
EXIT;
```

Remplacez 'password' par un mot de passe sécurisé de votre choix.

### Création de la Table messages

Connectez-vous à votre base de données avec le nouvel utilisateur :

```bash
mysql -u ros2 -p
```

Sélectionnez la base de données et créez la table :

```sql
USE rosdb;s

CREATE TABLE messages (
  id INT AUTO_INCREMENT PRIMARY KEY,
  content TEXT,
  timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

DESCRIBE messages;
EXIT;
```

## Configuration de ROS 2

### Installation des bibliothèques MySQL pour C++

Pour permettre à votre application C++ de se connecter à MySQL, vous devez installer libmysqlclient-dev :

```bash
sudo apt-get install libmysqlclient-dev
```

### Compilation et lancement du neux ROS

Naviguez vers votre espace de travail ROS 2 et compilez votre package avec le programme my_node.cpp et son CMakeLists.txt.

Le fichier my_node.cpp contient le code source d'un nœud ROS 2 qui souscrit au topic /test et enregistre chaque message reçu dans une base de données MySQL :

```bash
ros2 run my_robot_package my_node
```

## Test de l'Intégration

### Publiez un message sur le topic /test :

```bash
ros2 topic pub /test std_msgs/msg/String "data: 'Hello ROS 2'"
```

### Vérifiez que le message a été enregistré dans la base de données :

```bash
mysql -u ros2 -p

USE rosdb;

SELECT * FROM messages;

EXIT;
```

## Conclusion

Vous devriez maintenant être en mesure de voir les messages publiés sur le topic /test enregistrés dans votre base de données MySQL.
