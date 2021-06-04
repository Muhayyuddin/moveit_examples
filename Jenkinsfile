pipeline {
    agent any

    stages {
        stage('checkout') {
            steps {
                echo 'Hello World'
            }
        }
         stage('Build') {
            steps {
                echo 'Build'
            }
        }
         stage('Deploy') {
            steps {
                sh 'rm -rf RepoB'
                echo 'Deploy'
            }
        }
    }
}
