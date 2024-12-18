/* Copyright 2022 The MathWorks, Inc. */

/**
 * @file
 * Header file for DBoW2 ORBDatabase wrapper.
 */

#ifndef INVERTED_IMAGE_INDEX_HPP
#define INVERTED_IMAGE_INDEX_HPP

#ifdef _MSC_VER
#pragma warning(disable : 4244 4263 4267 4946)
#endif

#include <vector>
#include <string>

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "DBoW2/DBoW2.h"
    #include "DBoW2/QueryResults.h"
#else
    #include "DBoW2.h"
    #include "QueryResults.h"
#endif

namespace vision {
    namespace vslam {
        class InvertedImageIndex {
        public:
            /**
             * @brief Ctor
             *
             * @param[in]   Name of the file containing OrbVocabulary. File 
             *              extension is .yml.gz created using class  
             *              OrbVocabulary from 3p/DBoW2.
             *
             */
            InvertedImageIndex(const std::string& orbVocabularyFilename) 
                : my_vocabulary(), 
                    my_database(false, 0) {
                        my_vocabulary.loadFromBinaryFile(orbVocabularyFilename);
                        my_database.setVocabulary(my_vocabulary);
                    }
            
            /**
             * @brief Add features of image to index database 
             *
             * @param[in]  imageFeatures     Vector of features of an image
             * @return     Unique index generated by database of the new entry
             *
             */
            DBoW2::EntryId addImageFeatures(const std::vector<cv::Mat>& imageFeatures);
    
            /**
             * @brief 
             *
             * @param[in]  imageFeatures    Vector of features of an image
             * @param[in]  maxResults       Maximum number of results to return
             * @return     Vector of query results containing entry Ids and scores
             *
             */
            DBoW2::QueryResults retrieveImages(const std::vector<cv::Mat>& imageFeatures, const int maxResults);
    
        private:
            OrbVocabulary my_vocabulary;
            OrbDatabase my_database;
        };
    } // end namespace vslam
} // end namespace vision

#endif /* INVERTED_IMAGE_INDEX_HPP */
